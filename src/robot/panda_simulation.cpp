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

namespace gazebo
{
    class PandaSimulation : public ModelPlugin
    {
   public: void Load(physics::ModelPtr model, sdf::ElementPtr /*_sdf*/)
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
            bool added = false;
            if( true
                && joint->LowerLimit(0u) != joint->UpperLimit(0u)
                && joint->LowerLimit(0u) != 0
                && !std::isnan(joint->LowerLimit(0u))
                && !std::isnan(joint->UpperLimit(0u))
            )
            {
                joints_.push_back(joint);
                actuated_joint_names_.push_back(joint->GetName());
                added = true;
            }
/*
            ROS_INFO_STREAM( "[GazeboModel \'" << model->GetName() << "\'] " << (added ? "Adding":"Not adding")
                << (added ? "" : " fixed/invalid") << " joint " << joint->GetName()
                << " type (" << joint->GetType() << ")"
                << " lower limit " << joint->LowerLimit(0u)
                << " upper limit " << joint->UpperLimit(0u)
                << " Velocity lower limit " << joint->GetVelocityLimit(0u)
                << " Velocity upper limit " << joint->GetVelocityLimit(0u)
                << '\n');
                */
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
        ros::NodeHandle node_handle("/velocity_qp");
        
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

        elapsed_time_ = ros::Duration(0.0);
        
        ROS_WARN_STREAM("SIMULATED ROBOT");
        return;
    }
    
     bool loadWorld()
    {
        auto world = ::gazebo::physics::get_world();
        if(!world)
        {
            std::cerr << "[GazeboModel::" << getName() << "] " << "Could not load gazebo world" << '\n';
            return false;
        }
        world_ = world;
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        world_begin_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PandaSimulation::worldUpdateBegin, this));
        
         world_end_ = event::Events::ConnectWorldUpdateEnd(std::bind(&PandaSimulation::worldUpdateEnd,this));
        return true;
    }
    
    double getIterations()
    {
            return world_->Iterations();
    }
    
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
    
    const Eigen::Vector3d& getGravity() const
    {
        assertModelLoaded();
        return gravity_vector_;
    }

    const std::vector<std::string>& getActuatedJointNames() const
    {
        assertModelLoaded();
        return actuated_joint_names_;
    }

    const Eigen::Matrix<double,6,1>& getBaseVelocity() const
    {
        assertModelLoaded();
        return current_base_vel_;
    }

    const Eigen::Affine3d& getWorldToBaseTransform() const
    {
        assertModelLoaded();
        return current_world_to_base_;
    }

    const Eigen::VectorXd& getJointPositions() const
    {
        assertModelLoaded();
        return current_joint_positions_;
    }

    const Eigen::VectorXd& getJointVelocities() const
    {
        assertModelLoaded();
        return current_joint_velocities_;
    }

    void setJointGravityTorques(const Eigen::VectorXd& gravity_torques)
    {
        assertModelLoaded();
        if (gravity_torques.size() != joint_gravity_torques_.size())
            throw std::runtime_error("Provided gravity torques do not match the model's dofs");
        joint_gravity_torques_ = gravity_torques;
    }

    const Eigen::VectorXd& getJointExternalTorques() const
    {
        assertModelLoaded();
        return current_joint_external_torques_;
    }

    const Eigen::VectorXd& getJointMeasuredTorques() const
    {
        assertModelLoaded();
        return current_joint_measured_torques_;
    }

    void setJointTorqueCommand(const Eigen::VectorXd& joint_torque_command)
    {
        assertModelLoaded();
        joint_torque_command_ = joint_torque_command;
    }
    
        const std::string& getName() const
    {
        assertModelLoaded();
        return name_;
    }

    const std::string& getBaseName()
    {
        assertModelLoaded();
        return base_name_;
    }
    
    void setBrakes(bool enable)
    {
        brakes_ = enable;
    }

    int getNDof() const
    {
        assertModelLoaded();
        return ndof_;
    }
    
    // Called by the world update start event
    public: 
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
        
        
        joint_command_ = qp.update(getJointPositions(),getJointVelocities(),period);

        auto g = world_->Gravity();
        gravity_vector_[0] = g[0];
        gravity_vector_[1] = g[1];
        gravity_vector_[2] = g[2];

        if (control_level == "velocity")
        {
            for(int i=0 ; i < ndof_ ; ++i)
                joints_[i]->SetVelocity(0, joint_command_[i]);
            
        }
        else if (control_level == "torque")
        {
            for(int i=0 ; i < ndof_ ; ++i)
                joints_[i]->SetForce(0, joint_command_[i]);
            
        }
    }
    
    void executeAfterWorldUpdate(std::function<void(uint32_t,double,double)> callback)
    {
        callback_ = callback;
    }
    
    void worldUpdateEnd()
    {
        for(int i=0 ; i < ndof_ ; ++i)
        {
            auto joint = joints_[i];
        
            current_joint_positions_[i] = joint->Position(0);
            current_joint_velocities_[i] = joint->GetVelocity(0);
            current_joint_external_torques_[i] = joint->GetForce(0); // WARNING: This is the  external estimated force (= force applied to the joint = torque command from user)

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
    
    void setModelConfiguration(const std::vector<std::string>& joint_names,const std::vector<double>& joint_positions)
    {
    assertModelLoaded();
    if (joint_names.size() != joint_positions.size())
    {
        std::cerr << "[GazeboModel \'" << getName() << "\'] " << "joint_names lenght should be the same as joint_positions : " << joint_names.size() << " vs " << joint_positions.size() << '\n';
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
        private: physics::ModelPtr model;
                 physics::WorldPtr world_;
        void assertModelLoaded() const
            {
                if (!model_)
                {
                    ROS_ERROR_STREAM("Gazebo Model is no Loaded");
                    throw std::runtime_error("[GazeboModel] Model is not loaded");
                }
            }
    
        public: Eigen::VectorXd q, qd, joint_command_;
                std::vector<std::string> jn;
                std::vector<double> jp;
                gazebo::common::Time time;
                gazebo::common::PID pid;
                int i;
                ros::Duration period,elapsed_time_;
                physics::Joint_V joints_;
                physics::Link_V links_;
                std::string name_;
                int ndof_ = 0;
                Eigen::VectorXd current_joint_positions_;
                Eigen::VectorXd current_joint_velocities_;
                Eigen::VectorXd joint_gravity_torques_;
                Eigen::VectorXd current_joint_external_torques_;
                Eigen::VectorXd joint_torque_command_;
                Eigen::VectorXd current_joint_measured_torques_;
                gazebo::event::ConnectionPtr world_begin_;
                gazebo::event::ConnectionPtr world_end_;
                std::vector<std::string> actuated_joint_names_;
                physics::ModelPtr model_;
                Eigen::Vector3d gravity_vector_;
                Eigen::Matrix<double,6,1> current_base_vel_;
                Eigen::Affine3d current_world_to_base_;
                std::string base_name_;
                std::function<void(uint32_t,double,double)> callback_;
                bool brakes_;
                double old_time ;
                Controller::Controller qp;
                std::string control_level;
                ros::Publisher joint_states_pub_;
                sensor_msgs::JointState joint_states_;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PandaSimulation)
}
