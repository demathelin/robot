#include "velocity_qp/controller/controller.h"

using namespace KDL;
using namespace std;

namespace Controller{
bool Controller::Init(ros::NodeHandle& node_handle, Eigen::VectorXd q_init, Eigen::VectorXd qd_init) 
{
    ROS_WARN("Using velocity_qp controller !!! ");

    //--------------------------------------
    // INITIALIZE PUBLISHERS
    //--------------------------------------
    init_publishers(node_handle);
    
    //--------------------------------------
    // LOAD PARAMETERS
    //--------------------------------------
    load_parameters();
    
    //--------------------------------------
    // LOAD ROBOT
    //--------------------------------------
    if (!load_robot(node_handle))
        return false;
    
    //--------------------------------------
    // INITIALIZE VARIABLES
    //--------------------------------------  
    J_.resize(chain.getNrOfJoints());
    M_.resize(chain.getNrOfJoints());

    H_.resize(number_of_variables,number_of_variables);
    g_.resize(number_of_variables);
    lb_.resize(number_of_variables);
    ub_.resize(number_of_variables);
    A_.resize(number_of_constraints_,number_of_variables);
    lbA_.resize(number_of_constraints_);
    ubA_.resize(number_of_constraints_);
    joint_velocity_out_.resize(dof);
    gravity_kdl.resize(dof);
    play_traj_ = false;
    t_traj_curr = 0;
    dir_ = KDL::Vector(0.0,1.0,0.0); // Initialize direction vector
    
    //--------------------------------------
    // QPOASES
    //--------------------------------------
    qpoases_solver_.reset(new qpOASES::SQProblem(number_of_variables,number_of_constraints_,qpOASES::HST_POSDEF));

    // QPOases options
    qpOASES::Options options;
    // This options enables regularisation (required) and disable
    // some checks to be very fast !
    // options.setToDefault();
    options.setToMPC(); // setToReliable() // setToDefault()
    options.enableRegularisation = qpOASES::BT_FALSE; // since we specify the type of Hessian matrix, we do not need automatic regularisation
    options.enableEqualities = qpOASES::BT_TRUE; // Specifies whether equalities shall be  always treated as active constraints.
    qpoases_solver_->setOptions( options );
    qpoases_solver_->setPrintLevel(qpOASES::PL_NONE); // PL_HIGH for full output, PL_NONE for... none
    
    q_in.q.data = q_init;
    q_in.qdot.data = qd_init;
    fksolver_->JntToCart(q_in.q,X_curr_,8);
    
    //--------------------------------------
    // BUILD TRAJECTORY 
    //--------------------------------------
    BuildTrajectory(X_curr_);
    t_traj_curr += 0.001;
    return true;
}

std::tuple<Eigen::VectorXd, Eigen::VectorXd>Controller::update(Eigen::VectorXd q, Eigen::VectorXd qd, const ros::Duration& period)
{
    double time_dt = period.toSec();
    
    //Get robot current state
    q_in.q.data = q;
    q_in.qdot.data = qd;

    chainjacsolver_->JntToJac(q_in.q,J_,8);
    fksolver_->JntToCart(q_in.q,X_curr_,8);
    fksolvervel_->JntToCart(q_in,Xd_curr_,8);
    dynModelSolver_->JntToMass(q_in.q,M_);
    dynModelSolver_->JntToGravity(q_in.q, gravity_kdl);

    // Increment trajectory
    if (play_traj_)
    {
        ROS_INFO_ONCE(" Playing trajectory " );
        t_traj_curr += 0.001;
        if (t_traj_curr > ctraject->Duration())
        {
            t_traj_curr = init_dur;
        }

    }
    
    X_traj_ = ctraject->Pos(t_traj_curr);
    
//     ROS_ERROR_STREAM_ONCE(" X_traj_: " << X_traj_.p(0) << " " <<  X_traj_.p(1) << " "  << X_traj_.p(2));
    
    // Proportionnal controller 
    X_err_ = diff( X_curr_ , X_traj_ ); 
    tf::twistKDLToEigen(X_err_,x_err);
    xd_des_ = p_gains_.cwiseProduct(x_err);
        
    //--------------------------------------
    // QP SOLVER  
    //--------------------------------------
    J = J_.data;
    M = M_.data;
    
    
    H_ =  2.0 * regularisation_weight_ * Eigen::MatrixXd::Identity(7,7);
    g_ = -2.0 * regularisation_weight_ * p_gains_qd_.cwiseProduct((q_mean_ - q));
    

    H_ +=  2.0 *  J.transpose() * J;
    g_ += -2.0 *  J.transpose() * xd_des_;
    
    double horizon_dt = 15 * time_dt;
    
    ub_ = qd_max_;
    lb_ = qd_min_;
  
    A_.block(0,0,7,7) = horizon_dt * Eigen::MatrixXd::Identity(7,7);    
    ubA_.segment(0,7) = ul.data - q;
    lbA_.segment(0,7) = ll.data - q;
    
    //Get direction of motion
    X_traj_next = ctraject->Pos(t_traj_curr + 0.001); //Get next point along the trajectory
    if (X_traj_next != X_traj_)
    {
        dir_ = KDL::diff(X_traj_next.p,X_traj_.p); 
        double dir_n = dir_.Normalize();
        if (dir_n !=0) 
            tf::vectorKDLToEigen(dir_,u);
    }
    
    Lambda_ = ( J *  M.inverse() * J.transpose() ).inverse();
    m_u = u.transpose() * Lambda_.block(0,0,3,3) * u;
    
    //ec_lim as a function of the distance between the human and the robot worskspace
    double human_min_dist_= 0;
    double human_max_dist_= 3.5;
    double ec_safe = 0;
    double ec_max = 1;
    if (distance_to_contact <= human_min_dist_)
      ec_lim = ec_safe;
    if (distance_to_contact >= human_max_dist_)
      ec_lim = ec_max;
    if ((distance_to_contact < human_max_dist_)&&(distance_to_contact >human_min_dist_))
      ec_lim = ec_safe + (distance_to_contact-human_min_dist_) * (ec_max-ec_safe)/(human_max_dist_-human_min_dist_);
  
    
    
    // Kinetic energy constraint 
    //-sqrt(ec_max) < sqrt(0.5 m_u) v_u  < sqrt(ec_max);
    
    A_.block(dof,0,1,dof) = sqrt(0.5*m_u) * u.transpose() * J.block(0,0,3,7);
    ubA_(dof) =  sqrt(ec_lim);
    lbA_(dof) = -sqrt(ec_lim);

    
    // number of allowed compute steps
    int nWSR = 1e6;

    // Let's compute !
    qpOASES::returnValue ret;
    static bool qpoases_initialized = false;

    if(!qpoases_initialized){
        // Initialise the problem, once it has found a solution, we can hotstart
        ret = qpoases_solver_->init(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

        // Keep init if it didn't work
        if(ret == qpOASES::SUCCESSFUL_RETURN)
        qpoases_initialized = true;
    }
    else{
        // Otherwise let's reuse the previous solution to find a solution faster
        ret = qpoases_solver_->hotstart(H_.data(),g_.data(),A_.data(),lb_.data(),ub_.data(),lbA_.data(),ubA_.data(),nWSR);

        if(ret != qpOASES::SUCCESSFUL_RETURN)
        qpoases_initialized = false;
    }
    
    // Zero velocity if no solution found
    joint_velocity_out_.setZero();

    if(ret == qpOASES::SUCCESSFUL_RETURN)
        qpoases_solver_->getPrimalSolution(joint_velocity_out_.data());
    else
        ROS_WARN_STREAM("QPOases failed! Sending zero velocity");
    
    do_publishing();


    return std::make_tuple(joint_velocity_out_, gravity_kdl.data);
    // return joint_velocity_out_;
  
}

void Controller::BuildTrajectory(KDL::Frame X_curr_)
{
    ctraject = new Trajectory_Composite();

    KDL::Frame frame1,frame2,frame3;
    frame1 = X_curr_;

    frame2 = KDL::Frame( frame1.M, KDL::Vector(0.4,0.25,0.4)  );

    frame3 = KDL::Frame( frame1.M, KDL::Vector(0.4,-0.25,0.4) );


    double radius = 0.01;
    double eqradius = 0.05;
    double vmax = 0.5;
    double accmax = 0.5;
    path = new KDL::Path_RoundedComposite(radius,eqradius,new KDL::RotationalInterpolation_SingleAxis());
    path->Add(frame1);
    path->Add(frame2);
    path->Finish();

    velpref = new KDL::VelocityProfile_Trap(vmax,accmax);
    velpref->SetProfile(0,path->PathLength());
    init_dur = velpref -> Duration();
    ctraject -> Add (new KDL::Trajectory_Segment(path, velpref));

    ctraject -> Add (new KDL::Trajectory_Stationary(1,frame2));

    path = new KDL::Path_RoundedComposite(radius,eqradius,new KDL::RotationalInterpolation_SingleAxis());
    path->Add(frame2);
    path->Add(frame3);
    path->Finish();

    velpref = new KDL::VelocityProfile_Trap(vmax,accmax);
    velpref->SetProfile(0,path->PathLength());
    ctraject -> Add (new KDL::Trajectory_Segment(path, velpref));

    ctraject -> Add (new KDL::Trajectory_Stationary(1,frame3));

    path = new KDL::Path_RoundedComposite(radius,eqradius,new KDL::RotationalInterpolation_SingleAxis());
    path->Add(frame3);
    path->Add(frame2);
    path->Finish();

    velpref = new KDL::VelocityProfile_Trap(vmax,accmax);
    velpref->SetProfile(0,path->PathLength());
    ctraject -> Add (new KDL::Trajectory_Segment(path, velpref));

    ROS_INFO_STREAM(" Trajectory computed " );
   
}

void Controller::init_publishers(ros::NodeHandle& node_handle){
    //Realtime safe publishers
    pose_array_publisher.init(node_handle, "Pose_array", 1);
    path_publisher.init(node_handle, "Ros_Path", 1);    
    panda_rundata_publisher.init(node_handle, "panda_rundata", 1);
    human_workspace_dist_sub = node_handle.subscribe("/Laser_workspace_distance",1000, &Controller::distance_callback, this);

}

void Controller::load_parameters(){    
    ROS_INFO_STREAM ( "------------- Loading parameters -------------" );
    qd_min_.resize(7);
    getRosParam("/velocity_qp/qd_min_",qd_min_);
    qd_max_.resize(7);
    getRosParam("/velocity_qp/qd_max_",qd_max_);
    p_gains_.resize(6);
    getRosParam("/velocity_qp/p_gains_",p_gains_);
    d_gains_.resize(6);
    getRosParam("/velocity_qp/d_gains_",d_gains_);
    p_gains_qd_.resize(7);
    getRosParam("/velocity_qp/p_gains_qd_",p_gains_qd_);
    torque_max_.resize(7);
    getRosParam("/velocity_qp/torque_max_",torque_max_);
    qdd_max_.resize(7);
    getRosParam("/velocity_qp/qdd_max_",qdd_max_);
    q_mean_.resize(7);
    getRosParam("/velocity_qp/q_mean_",q_mean_);
    getRosParam("/velocity_qp/regularisation_weight_",regularisation_weight_);
    getRosParam("/velocity_qp/root_link_",root_link_);
    getRosParam("/velocity_qp/tip_link_",tip_link_);
    ROS_INFO_STREAM ( "------------- Parameters Loaded -------------" );
} 

bool Controller::load_robot(ros::NodeHandle& node_handle){

    dynamic_reconfigure_params_node_ = ros::NodeHandle("dynamic_params_node");
    dynamic_server_params_ = std::make_unique<dynamic_reconfigure::Server<velocity_qp::paramConfig>>(dynamic_reconfigure_params_node_);
    dynamic_server_params_->setCallback(boost::bind(&Controller::paramCallback, this, _1, _2));
    
    // get robot descritpion
    double timeout;
    node_handle.param("timeout", timeout, 0.005);
    std::string urdf_param;
    node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
    double eps = 1e-5;
    ik_solver.reset(new TRAC_IK::TRAC_IK(root_link_, tip_link_, urdf_param, timeout, eps));
    bool valid = ik_solver->getKDLChain(chain);

    if (!valid) {
        ROS_ERROR_STREAM("There was no valid KDL chain found");
        return false;
    }
    valid = ik_solver->getKDLLimits(ll,ul);
    if (!valid) {
        ROS_ERROR_STREAM("There were no valid KDL joint limits found");
        return false;
    }

    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    if(chain.getNrOfSegments() == 0)
        ROS_WARN("KDL chain empty !");

    ROS_INFO_STREAM("  Chain has "<<chain.getNrOfJoints()<<" joints");
    ROS_INFO_STREAM("  Chain has "<<chain.getNrOfSegments()<<" segments");

    for(unsigned int i=0; i<chain.getNrOfSegments(); ++i)
        ROS_INFO_STREAM("    "<<chain.getSegment(i).getName());

    fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain));
    fksolvervel_.reset(new KDL::ChainFkSolverVel_recursive(chain));
    chainjacsolver_.reset(new KDL::ChainJntToJacSolver(chain)); 
    dynModelSolver_.reset(new KDL::ChainDynParam(chain, KDL::Vector(0.,0.,-9.81)));
    dof = chain.getNrOfJoints();
    number_of_variables = dof;
    number_of_constraints_ = dof+1;
    ROS_INFO_STREAM ( "Number of variables : " << number_of_variables );
    ROS_INFO_STREAM ( "Number of constraints : " << number_of_constraints_);
    
    return true;
}

void Controller::do_publishing(){
    // Publishing
    tf::poseKDLToMsg(X_curr_,X_curr_msg_);
    tf::poseKDLToMsg(X_traj_,X_traj_msg_);
    tf::twistKDLToMsg(X_err_,X_err_msg_);
    tf::twistEigenToMsg(xd_des_,xd_des_msg_);
    if (panda_rundata_publisher.trylock()){
        panda_rundata_publisher.msg_.header.stamp = ros::Time::now();
        panda_rundata_publisher.msg_.header.frame_id = root_link_;
        panda_rundata_publisher.msg_.ec_max = ec_lim;
        panda_rundata_publisher.msg_.ec_pred = (A_.block(dof,0,1,dof) * joint_velocity_out_)(0)*(A_.block(dof,0,1,dof) * joint_velocity_out_)(0) ;            
        panda_rundata_publisher.msg_.t_traj_curr = t_traj_curr;
        panda_rundata_publisher.msg_.m_u = m_u;
        panda_rundata_publisher.msg_.X_curr = X_curr_msg_;
        panda_rundata_publisher.msg_.X_traj = X_traj_msg_;
        panda_rundata_publisher.msg_.X_err = X_err_msg_;
        panda_rundata_publisher.msg_.Xd_control = xd_des_msg_;
        panda_rundata_publisher.unlockAndPublish();
    }    
}

void Controller::paramCallback(velocity_qp::paramConfig& config, uint32_t) {
    
    play_traj_ = config.play_traj;
    distance_to_contact = config.human_distance;
    
}

void Controller::distance_callback(const std_msgs::Float32::ConstPtr& msg)
{
    distance_to_contact = msg->data;
//     new_distance_acquired = true;
}

}
