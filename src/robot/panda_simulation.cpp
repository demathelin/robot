#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <velocity_qp/controller/controller.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <boost/scoped_ptr.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chaindynparam.hpp>


namespace gazebo
{
    /**
    * \class Simulation class .
    */
    class PandaSimulation : public ModelPlugin
    {
   public:
   
   /**
    * \fn  void Load.
    * \brief Load the robot model from gazebo.
    * \param physics::ModelPtr model the robot model.
    */
    void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/)
    {
        if(!model)
        {
            std::cerr << "Model is invalid" << '\n';
            return ;
        }
        
        if(!loadWorld())
            return;
        
        joints_.clear();
        actuated_joint_names_.clear();
        for(auto joint : model->GetJoints() )
        {
            // Provide feedback to get the internal torque
            joint->SetProvideFeedback(true);
            // Not adding fixed joints
            if( true
                && joint->LowerLimit(0u) != joint->UpperLimit(0u)
                && joint->LowerLimit(0u) != 0
                && !std::isnan(joint->LowerLimit(0u))
                && !std::isnan(joint->UpperLimit(0u))
            )
            {
                joints_.push_back(joint);
                actuated_joint_names_.push_back(joint->GetName());
            }
        }

        if(actuated_joint_names_.size() == 0)
        {
            std::cerr << "[GazeboModel \'" << model->GetName() << "\'] " << "Could not get any actuated joint for model " << model->GetName() << '\n';
            return ;
        }
        
        model_ = model;

            std::map<std::string, double> init_joint_positions;
        if(!ros::param::get("/init_joint_positions",init_joint_positions))
        {
        ROS_WARN_STREAM("" << ros::this_node::getName() << "Could not find init_joint_positions in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        }
        else
        {
            ndof_ = actuated_joint_names_.size();
            current_joint_positions_.setZero(ndof_);
            for(auto e : init_joint_positions)
            {
                jn.push_back(e.first);
                jp.push_back(e.second);
                for(int i=0; i< actuated_joint_names_.size();++i)
                {
                    if (e.first == actuated_joint_names_.at(i))
                    {
                        current_joint_positions_(i) = e.second;
                    }
                }
            }
            setModelConfiguration(jn,jp);
        }
   
        
        links_ = model->GetLinks();
        name_ = model->GetName();
        
        
        joint_gravity_torques_.setZero(ndof_);
        current_joint_velocities_.setZero(ndof_);
        current_joint_external_torques_.setZero(ndof_);
        joint_torque_command_.setZero(ndof_);
        current_joint_measured_torques_.setZero(ndof_);
    

        current_joint_velocities_ = getJointVelocities();
        joint_command_.resize(7);        
        ros::NodeHandle node_handle;
        
        if (!node_handle.getParam("/velocity_qp/control_level", control_level)) {
            ROS_ERROR_STREAM("Could not read parameter control_level");
        }
        if (control_level == "velocity")
        {
             ROS_WARN_STREAM("Velocity controlled robot");
        }
        else if (control_level == "torque")
        {
           ROS_WARN_STREAM("Torque controlled robot");
        }
        else{
        
            ROS_ERROR_STREAM("Wrong control level definition ");
        }
        
        joint_states_pub_ = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1, true);
        joint_states_.name = getActuatedJointNames();
        joint_states_.position.resize(ndof_);
        joint_states_.velocity.resize(ndof_);
        joint_states_.effort.resize(ndof_);

        qp.Init(node_handle,current_joint_positions_,current_joint_velocities_);

         
        // get robot descritpion
        double timeout;
        node_handle.param("timeout", timeout, 0.005);
        std::string urdf_param;
        node_handle.param("urdf_param", urdf_param, std::string("/robot_description"));
        double eps = 1e-5;
        std::string root_link_,tip_link_;
        node_handle.getParam("/velocity_qp/root_link_",root_link_);
        node_handle.getParam("/velocity_qp/tip_link_",tip_link_);
        KDL::Chain chain;
        ik_solver.reset(new TRAC_IK::TRAC_IK(root_link_, tip_link_, urdf_param, timeout, eps));
        ik_solver->getKDLChain(chain);

        for(unsigned int i=0; i<chain.getNrOfSegments(); ++i)
            ROS_INFO_STREAM("    "<<chain.getSegment(i).getName());

        dynModelSolver_.reset(new KDL::ChainDynParam(chain, KDL::Vector(0.,0.,-9.81)));
            
        ROS_WARN_STREAM("SIMULATED ROBOT");
        return;
    }
    
    /**
    * \fn  void loadWolrd.
    * \brief Load the gazebo world.
    */
    bool loadWorld()
    {
        auto world = ::gazebo::physics::get_world();
        if(!world)
        {
            std::cerr << "[GazeboModel::" << getName() << "] " << "Could not load gazebo world" << '\n';
            return false;
        }
        world_ = world;
        // Listen to the update event. This event is broadcast every.
        // simulation iteration.
        world_begin_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PandaSimulation::worldUpdateBegin, this));
        
         world_end_ = event::Events::ConnectWorldUpdateEnd(std::bind(&PandaSimulation::worldUpdateEnd,this));
        return true;
    }
    
    /**
    * \fn  double getIterations.
    * \brief Get the number of iteration since the begining of the simulation.
    */
    double getIterations()
    {
            return world_->Iterations();
    }
    
    /**
    * \fn  void print.
    * \brief Print some information from gazebo.
    * \return Gazebo model name.
    * \return Robot joints name
    * \return Robot actuated joints name.
    * \return Robot links name.
    * \return Robot state.
    */
    void print() const
    {
        if(!model_)
        {
            std::cout << "[GazeboModel] Model is not loaded." << '\n';
            return;
        }

        std::cerr << "[GazeboModel::" << model_->GetName() << "]" << '\n';
        
        std::cout << "  Joints (" << joints_.size() << ")" << '\n';
        for(unsigned int i=0; i < joints_.size() ; ++i)
            std::cout << "     Joint " << i << ": '" << joints_[i]->GetName() << "'" << '\n';
        
        std::cout << "  Actuated joints (" << actuated_joint_names_.size() << ")" << '\n';
        for(unsigned int i=0; i < actuated_joint_names_.size() ; i++)
            std::cout << "     Actuated joint " << i << ": '" << actuated_joint_names_[i] << "'" << '\n';

        std::cout << "  Links (" << links_.size() << ")" << '\n';
        for(unsigned int i=0; i < links_.size() ; i++)
            std::cout << "      Link " << i << ": '" << links_[i]->GetName() << "'" << '\n';
        
        printState();
    }
    
    /**
    * \fn  void printState.
    * \brief Print some information of the robot in gazebo.
    * \return Gazebo model name.
    * \return Gravity vector.
    * \return Velocity of the base frame.
    * \return Transform between the world frame and the base frame.
    * \return Robot joint position.
    * \return Robot joint velocity.
    * \return Robot external torques.
    * \return Robot measured torques.
    */
     void printState() const
    {
        if(!model_)
        {
            std::cout << "[GazeboModel] Model is not loaded." << '\n';
            return;
        }
        
        std::cout << "[GazeboModel \'" << model_->GetName() << "\'] State :" << '\n';
        std::cout << "- Gravity "                   <<  getGravity().transpose()                << '\n';
        std::cout << "- Base velocity\n"            << getBaseVelocity().transpose()           << '\n';
        std::cout << "- Tworld->base\n"             << getWorldToBaseTransform().matrix()      << '\n';
        std::cout << "- Joint positions "           << getJointPositions().transpose()         << '\n';
        std::cout << "- Joint velocities "          << getJointVelocities().transpose()        << '\n';
        std::cout << "- Joint external torques "    << getJointExternalTorques().transpose()   << '\n';
        std::cout << "- Joint measured torques "    << getJointMeasuredTorques().transpose()   << '\n';
    }
    
    /**
    * \fn Eigen::Vector3d& getGravity.
    * \brief Get the gravity vector.
    * \return Gravity vector.
    */
    const Eigen::Vector3d& getGravity() const
    {
        assertModelLoaded();
        return gravity_vector_;
    }

    /**
    * \fn const std::vector<std::string>& getActuatedJointNames.
    * \brief Get the actuated joint names.
    * \return Actuated joints names.
    */
    const std::vector<std::string>& getActuatedJointNames() const
    {
        assertModelLoaded();
        return actuated_joint_names_;
    }

    /**
    * \fn const std::vector<std::string>& getBaseVelocity.
    * \brief Get the velocity of the base frame.
    * \return The velocity of the base frame.
    */
    const Eigen::Matrix<double,6,1>& getBaseVelocity() const
    {
        assertModelLoaded();
        return current_base_vel_;
    }

    /**
    * \fn Eigen::Affine3d& getWorldToBaseTransform.
    * \brief Get the transform from the world frame to the base frame.
    * \return The transform from the world frame to the base frame.
    */
    const Eigen::Affine3d& getWorldToBaseTransform() const
    {
        assertModelLoaded();
        return current_world_to_base_;
    }

    /**
    * \fn Eigen::VectorXd& getJointPositions.
    * \brief Get the robot current joints positions.
    * \return The robot current joints positions.
    */
    const Eigen::VectorXd& getJointPositions() const
    {
        assertModelLoaded();
        return current_joint_positions_;
    }

    /**
    * \fn Eigen::VectorXd& getJointVelocities.
    * \brief Get the robot current joints velocity.
    * \return The robot current joints velocity.
    */
    const Eigen::VectorXd& getJointVelocities() const
    {
        assertModelLoaded();
        return current_joint_velocities_;
    }

    /**
    * \fn void setJointGravityTorques.
    * \brief Set the joint gravity vector.
    * \param const Eigen::VectorXd& gravity_torques. The new gravity vector.
    */
    void setJointGravityTorques(const Eigen::VectorXd& gravity_torques)
    {
        assertModelLoaded();
        if (gravity_torques.size() != joint_gravity_torques_.size())
            throw std::runtime_error("Provided gravity torques do not match the model's dofs");
        joint_gravity_torques_ = gravity_torques;
    }

    /**
    * \fn const Eigen::VectorXd& getJointExternalTorques.
    * \brief Get the joint external torques applied on the joints.
    * \return The external torques applied on the joints.
    */
    const Eigen::VectorXd& getJointExternalTorques() const
    {
        assertModelLoaded();
        return current_joint_external_torques_;
    }

    /**
    * \fn const Eigen::VectorXd& getJointMeasuredTorques.
    * \brief Get the joint torques applied on the joints.
    * \return The torques applied on the joints.
    */
    const Eigen::VectorXd& getJointMeasuredTorques() const
    {
        assertModelLoaded();
        return current_joint_measured_torques_;
    }

    /**
    * \fn void setJointTorqueCommand.
    * \brief Set the joint torque command.
    * \param const Eigen::VectorXd& joint_torque_command. The desired joint torque command to apply on the robot.
    */
    void setJointTorqueCommand(const Eigen::VectorXd& joint_torque_command)
    {
        assertModelLoaded();
        joint_torque_command_ = joint_torque_command;
    }
    
    /**
    * \fn void setJointTorqueCommand.
    * \brief Get the name of the model.
    * \return The name of the model.
    */
    const std::string& getName() const
    {
        assertModelLoaded();
        return name_;
    }

    /**
    * \fn std::string& getBaseName.
    * \brief Get the name of the robot base.
    * \return The name of the robot base.
    */
    const std::string& getBaseName()
    {
        assertModelLoaded();
        return base_name_;
    }
    
    /**
    * \fn void setBrakes.
    * \brief pause the robot.
    * \param bool enable. Enable pause.
    */
    void setBrakes(bool enable)
    {
        brakes_ = enable;
    }

    /**
    * \fn int getNDof.
    * \brief Get number of degrees of freedom of the robot.
    * \param bool enable. Enable pause.
    */
    int getNDof() const
    {
        assertModelLoaded();
        return ndof_;
    }
    
    // Called by the world update start event
    public: 
    /**
    * \fn worldUpdateBegin
    * \brief Called by the world update start event
    */
    void worldUpdateBegin()
    {
        period = ros::Duration(world_->Physics()->GetMaxStepSize());
        brakes_ = false;
        model_->SetEnabled(!brakes_); // Enable the robot when brakes are off

        joint_states_.header.stamp = ros::Time::now();
        Eigen::VectorXd::Map(joint_states_.position.data(),joint_states_.position.size()) = getJointPositions();
        Eigen::VectorXd::Map(joint_states_.velocity.data(),joint_states_.velocity.size()) = getJointVelocities();
        Eigen::VectorXd::Map(joint_states_.effort.data(),joint_states_.effort.size()) = getJointMeasuredTorques();


        joint_states_pub_.publish(joint_states_);
        KDL::JntArray gravity_kdl,q_kdl;
        q_kdl.data = q;
        gravity_kdl.resize(ndof_);
        dynModelSolver_->JntToGravity(q_kdl, gravity_kdl);
        Eigen::VectorXd gravity_comp;
        gravity_comp.resize(7);
        std::tie(joint_command_, gravity_comp) = qp.update(getJointPositions(),getJointVelocities(),period);

        auto g = world_->Gravity();
        gravity_vector_[0] = g[0];
        gravity_vector_[1] = g[1];
        gravity_vector_[2] = g[2];

        if (control_level == "velocity")
        {
            for(int i=0 ; i < ndof_ ; ++i)
            {
                joints_[i]->SetVelocity(0, joint_command_[i]);
                joints_[i]->SetForce(0, gravity_kdl.data[i]);
            }
            
        }
        else if (control_level == "torque")
        {
            for(int i=0 ; i < ndof_ ; ++i)
                joints_[i]->SetForce(0, joint_command_[i]);
            
        }
    }
    
    /**
    * \fn executeAfterWorldUpdate
    * \brief Called after the world update
    */
    void executeAfterWorldUpdate(std::function<void(uint32_t,double,double)> callback)
    {
        callback_ = callback;
    }
    
    /**
    * \fn worldUpdateEnd
    * \brief Called at the end of the world update
    */
    void worldUpdateEnd()
    {
        for(int i=0 ; i < ndof_ ; ++i)
        {
            auto joint = joints_[i];
        
            current_joint_positions_[i] = joint->Position(0);
            current_joint_velocities_[i] = joint->GetVelocity(0);
            current_joint_external_torques_[i] = joint->GetForce(0); // WARNING: This is the  external estimated force 
                                                                     // (= force applied to the joint = torque command from user)

            auto w = joint->GetForceTorque(0u);
            auto a = joint->LocalAxis(0u);
            current_joint_measured_torques_[i] = a.Dot(w.body1Torque);
        }

        if(callback_)
        {
            double sim_time = world_->SimTime().Double();
            double dt = world_->Physics()->GetMaxStepSize();
            callback_(getIterations(),sim_time,dt);
        }
    }
    
    /**
    * \fn setModelConfiguration
    * \brief Set the robot configuration (Useful during the initialization)
    * \param const std::vector<std::string>& joint_names vector containing the names of the joint to configure
    * \param const std::vector<double>& joint_positions vector containing the position of the joint to configure
    */

    void setModelConfiguration(const std::vector<std::string>& joint_names,const std::vector<double>& joint_positions)
    {
    assertModelLoaded();
    if (joint_names.size() != joint_positions.size())
    {
        std::cerr << "[GazeboModel \'" << getName() << "\'] " << "joint_names lenght should be the same as joint_positions : " 
        << joint_names.size() << " vs " << joint_positions.size() << '\n';
        return;
    }

    auto world = ::gazebo::physics::get_world();
    // make the service call to pause gazebo
    bool is_paused = world->IsPaused();
    if (!is_paused) world->SetPaused(true);

    std::map<std::string, double> joint_position_map;
    for (unsigned int i = 0; i < joint_names.size(); i++)
        joint_position_map[joint_names[i]] = joint_positions[i];

    model_->SetJointPositions(joint_position_map);

    // resume paused state before this call
    world->SetPaused(is_paused);

    return ;
}
        
        // Pointer to the model
        private: physics::ModelPtr model; /*!< @brief Gazebo Model pointer */  
                 physics::WorldPtr world_; /*!< @brief Gazebo World pointer */

        
    /**
    * \fn  void assertModelLoaded
    * \brief Check if the model is correctly loaded
    * \return Call runtime error if model not loaded
    */
        void assertModelLoaded() const
            {
                if (!model_)
                {
                    ROS_ERROR_STREAM("Gazebo Model is no Loaded");
                    throw std::runtime_error("[GazeboModel] Model is not loaded");
                }
            }
    
        public:
            Eigen::VectorXd q, qd, joint_command_;
            boost::shared_ptr<TRAC_IK::TRAC_IK> ik_solver; /*!< @brief Inverse kinematic solver pointer */
            std::vector<std::string> jn; /*!< @brief Joint name std vector */ 
            std::vector<double> jp; /*!< @brief Joint position std vector */  
            int i;
            ros::Duration period; /*!< @brief Refresh rate of Gazebo*/
            ros::Duration elapsed_time_; /*!< @brief elapsed time since begining of the simulation */  
            physics::Joint_V joints_; /*!< @brief Gazerbo joint vector */  
            physics::Link_V links_; /*!< @brief Gazerbo links vector */
            std::string name_;  /*!< @brief name of the model */
            int ndof_ = 0; /*!< @brief Number of degrees of freedom of the robot */
            Eigen::VectorXd current_joint_positions_; /*!< @brief Current joint position of the robot */
            Eigen::VectorXd current_joint_velocities_; /*!< @brief Current joint velocity of the robot */
            Eigen::VectorXd joint_gravity_torques_; /*!< @brief Joint torque induced by the gravity */
            Eigen::VectorXd current_joint_external_torques_; /*!< @brief Measured external torque on the robot joints */
            Eigen::VectorXd joint_torque_command_; /*!< @brief Joint torque command*/
            Eigen::VectorXd current_joint_measured_torques_; /*!< @brief Current torque applied on the robot joints */
            gazebo::event::ConnectionPtr world_begin_; /*!< @brief Pointer to connect to the begining of an update */
            gazebo::event::ConnectionPtr world_end_; /*!< @brief Pointer to connect to the end of an update */
            std::vector<std::string> actuated_joint_names_; /*!< @brief std vector of the name of the actuated joints */
            physics::ModelPtr model_; /*!< @brief Gazebo model of the robot */
            Eigen::Vector3d gravity_vector_; /*!< @brief Gravity vector (usually (0,0,-9.81)) */
            Eigen::Matrix<double,6,1> current_base_vel_; /*!< @brief Position of the robot base */
            Eigen::Affine3d current_world_to_base_; /*!< @brief Transform between the world pose to the robot base */
            std::string base_name_; /*!< @brief Name of the robot base */
            std::function<void(uint32_t,double,double)> callback_;
            bool brakes_; /*!< @brief Activate or not the brakes */
            Controller::Controller qp; /*!< @brief QP controller object */
            std::string control_level; /*!< @brief Control level, either "velocity" or "torque" */
            ros::Publisher joint_states_pub_; /*!< @brief Joint state publisher */
            sensor_msgs::JointState joint_states_; /*!< @brief Robot joint state */
            boost::scoped_ptr<KDL::ChainDynParam> dynModelSolver_; /*!< @brief Dynamic solver (to compute gravity torques) */
        private: event::ConnectionPtr updateConnection;  /*!< @brief Pointer to the update event connection */ 
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PandaSimulation)
}
