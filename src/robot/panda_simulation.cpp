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
        
        joints.clear();
        actuated_joint_names.clear();
        for(auto joint : model->GetJoints() )
        {
            // Provide feedback to get the internal torque
            joint->SetProvideFeedback(true);
            // Not adding fixed joints
            bool added = false;
            if( joint->GetType() != 16448)
            {
                joints.push_back(joint);
                actuated_joint_names.push_back(joint->GetName());
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

        if(actuated_joint_names.size() == 0)
        {
            std::cerr << "[GazeboModel \'" << model->GetName() << "\'] " << "Could not get any actuated joint for model " << model->GetName() << '\n';
            return ;
        }
        
        this->model = model;

            std::map<std::string, double> init_joint_positions;
        if(!ros::param::get("/init_joint_positions",init_joint_positions))
        {
        ROS_WARN_STREAM("" << ros::this_node::getName() << "Could not find init_joint_positions in namespace "
            << ros::this_node::getNamespace()
            << "/" << ros::this_node::getName());
        }
        else
        {
            ndof = actuated_joint_names.size();
            current_joint_positions.setZero(ndof);
            for(auto e : init_joint_positions)
            {
                jn.push_back(e.first);
                jp.push_back(e.second);
                for(int i=0; i< actuated_joint_names.size();++i)
                {
                    if (e.first == actuated_joint_names.at(i))
                    {
                        current_joint_positions(i) = e.second;
                    }
                }
            }
            setModelConfiguration(jn,jp);
        }
   
        
        links = model->GetLinks();
        name = model->GetName();
        
        
        joint_gravity_torques.setZero(ndof);
        current_joint_velocities.setZero(ndof);
        current_joint_external_torques.setZero(ndof);
        joint_torque_command.setZero(ndof);
        current_joint_measured_torques.setZero(ndof);
    
        current_joint_velocities = getJointVelocities();
        joint_command.resize(ndof);        
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
        
        
        joint_statespub = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1, true);
        joint_states.name = getActuatedJointNames();
        joint_states.position.resize(ndof);
        joint_states.velocity.resize(ndof);
        joint_states.effort.resize(ndof);

        qp.init(node_handle,current_joint_positions,current_joint_velocities);

        elapsed_time = ros::Duration(0.0);
        
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
        this->world = world;
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        world_begin = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PandaSimulation::worldUpdateBegin, this));
        
         world_end = event::Events::ConnectWorldUpdateEnd(std::bind(&PandaSimulation::worldUpdateEnd,this));
        return true;
    }
    
    double getIterations()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            return world->Iterations();
        #else
            return world->GetIterations();
        #endif
    }
    
    void print() const
    {
        if(!model)
        {
            std::cout << "[GazeboModel] Model is not loaded." << '\n';
            return;
        }

        std::cerr << "[GazeboModel::" << model->GetName() << "]" << '\n';
        
        std::cout << "  Joints (" << joints.size() << ")" << '\n';
        for(unsigned int i=0; i < joints.size() ; ++i)
            std::cout << "     Joint " << i << ": '" << joints[i]->GetName() << "'" << '\n';
        
        std::cout << "  Actuated joints (" << actuated_joint_names.size() << ")" << '\n';
        for(unsigned int i=0; i < actuated_joint_names.size() ; i++)
            std::cout << "     Actuated joint " << i << ": '" << actuated_joint_names[i] << "'" << '\n';

        std::cout << "  Links (" << links.size() << ")" << '\n';
        for(unsigned int i=0; i < links.size() ; i++)
            std::cout << "      Link " << i << ": '" << links[i]->GetName() << "'" << '\n';
        
        printState();
    }
    
     void printState() const
    {
        if(!model)
        {
            std::cout << "[GazeboModel] Model is not loaded." << '\n';
            return;
        }
        
        std::cout << "[GazeboModel \'" << model->GetName() << "\'] State :" << '\n';
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
        return gravity_vector;
    }

    const std::vector<std::string>& getActuatedJointNames() const
    {
        assertModelLoaded();
        return actuated_joint_names;
    }

    const Eigen::Matrix<double,6,1>& getBaseVelocity() const
    {
        assertModelLoaded();
        return current_base_vel;
    }

    const Eigen::Affine3d& getWorldToBaseTransform() const
    {
        assertModelLoaded();
        return current_world_to_base;
    }

    const Eigen::VectorXd& getJointPositions() const
    {
        assertModelLoaded();
        return current_joint_positions;
    }

    const Eigen::VectorXd& getJointVelocities() const
    {
        assertModelLoaded();
        return current_joint_velocities;
    }

    void setJointGravityTorques(const Eigen::VectorXd& gravity_torques)
    {
        assertModelLoaded();
        if (gravity_torques.size() != joint_gravity_torques.size())
            throw std::runtime_error("Provided gravity torques do not match the model's dofs");
        joint_gravity_torques = gravity_torques;
    }

    const Eigen::VectorXd& getJointExternalTorques() const
    {
        assertModelLoaded();
        return current_joint_external_torques;
    }

    const Eigen::VectorXd& getJointMeasuredTorques() const
    {
        assertModelLoaded();
        return current_joint_measured_torques;
    }

    void setJointTorqueCommand(const Eigen::VectorXd& new_joint_torque_command)
    {
        assertModelLoaded();
        joint_torque_command = new_joint_torque_command;
    }
    
        const std::string& getName() const
    {
        assertModelLoaded();
        return name;
    }

    const std::string& getBaseName()
    {
        assertModelLoaded();
        return base_name;
    }
    
    void setBrakes(bool enable)
    {
        brakes = enable;
    }

    int getNDof() const
    {
        assertModelLoaded();
        return ndof;
    }
    
    // Called by the world update start event
    public: 
    void worldUpdateBegin()
    {
        #if GAZEBO_MAJOR_VERSION > 8
            period = ros::Duration(world->Physics()->GetMaxStepSize());
        #else
            period = ros::Duration(world->GetPhysicsEngine()->GetMaxStepSize());
        #endif
        brakes = false;
        model->SetEnabled(!brakes); // Enable the robot when brakes are off

        joint_states.header.stamp = ros::Time::now();
        Eigen::VectorXd::Map(joint_states.position.data(),joint_states.position.size()) = getJointPositions();
        Eigen::VectorXd::Map(joint_states.velocity.data(),joint_states.velocity.size()) = getJointVelocities();
        Eigen::VectorXd::Map(joint_states.effort.data(),joint_states.effort.size()) = getJointMeasuredTorques();


        joint_statespub.publish(joint_states);
        
        
        joint_command = qp.update(getJointPositions(),getJointVelocities(),period);

        auto g = world->Gravity();
        gravity_vector[0] = g[0];
        gravity_vector[1] = g[1];
        gravity_vector[2] = g[2];

        if (control_level == "velocity")
        {
            for(int i=0 ; i < ndof ; ++i)
                joints[i]->SetVelocity(0, joint_command[i]);
            
        }
        else if (control_level == "torque")
        {
            for(int i=0 ; i < ndof ; ++i)
                joints[i]->SetForce(0, joint_command[i]);    
        }
    }
    
    void executeAfterWorldUpdate(std::function<void(uint32_t,double,double)> callback)
    {
        callback_ = callback;
    }
    
    void worldUpdateEnd()
    {
        for(int i=0 ; i < ndof ; ++i)
        {
            auto joint = joints[i];
        
            #if GAZEBO_MAJOR_VERSION > 8
                current_joint_positions[i] = joint->Position(0);
            #else
                current_joint_positions[i] = joint->GetAngle(0).Radian();
            #endif
            current_joint_velocities[i] = joint->GetVelocity(0);
            current_joint_external_torques[i] = joint->GetForce(0); // WARNING: This is the  external estimated force (= force applied to the joint = torque command from user)

            auto w = joint->GetForceTorque(0u);
            #if GAZEBO_MAJOR_VERSION > 8
                auto a = joint->LocalAxis(0u);
            #else
                auto a = joint->GetLocalAxis(0u);
            #endif
            current_joint_measured_torques[i] = a.Dot(w.body1Torque);
        }

        if(callback_)
        {
            #if GAZEBO_MAJOR_VERSION > 8
                double sim_time = world->SimTime().Double();
            #else
                double sim_time = world->GetSimTime().Double();
            #endif
            #if GAZEBO_MAJOR_VERSION > 8
                double dt = world->Physics()->GetMaxStepSize();
            #else
                double dt = world->GetPhysicsEngine()->GetMaxStepSize();
            #endif
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

    model->SetJointPositions(joint_position_map);

    // resume paused state before this call
    world->SetPaused(is_paused);

    return ;
}
        
        // Pointer to the model
        private: physics::ModelPtr model;
                 physics::WorldPtr world;
        void assertModelLoaded() const
            {
                if (!model)
                {
                    ROS_ERROR_STREAM("Gazebo Model is no Loaded");
                    throw std::runtime_error("[GazeboModel] Model is not loaded");
                }
            }
    
        public: Eigen::VectorXd q, qd, joint_command;
                std::vector<std::string> jn;
                std::vector<double> jp;
                gazebo::common::Time time;
                gazebo::common::PID pid;
                int i;
                ros::Duration period,elapsed_time;
                physics::Joint_V joints;
                physics::Link_V links;
                std::string name;
                int ndof = 0;
                Eigen::VectorXd current_joint_positions;
                Eigen::VectorXd current_joint_velocities;
                Eigen::VectorXd joint_gravity_torques;
                Eigen::VectorXd current_joint_external_torques;
                Eigen::VectorXd joint_torque_command;
                Eigen::VectorXd current_joint_measured_torques;
                gazebo::event::ConnectionPtr world_begin;
                gazebo::event::ConnectionPtr world_end;
                std::vector<std::string> actuated_joint_names;
                Eigen::Vector3d gravity_vector;
                Eigen::Matrix<double,6,1> current_base_vel;
                Eigen::Affine3d current_world_to_base;
                std::string base_name;
                std::function<void(uint32_t,double,double)> callback_;
                bool brakes;
                double old_time ;
                Controller::Controller qp;
                std::string control_level;
                ros::Publisher joint_statespub;
                sensor_msgs::JointState joint_states;
        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(PandaSimulation)
}
