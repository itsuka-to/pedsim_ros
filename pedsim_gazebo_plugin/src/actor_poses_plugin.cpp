/*
Created on Mon Dec  2

@author: mahmoud
*/

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

// #include <ros/ros.h>
// #include "ros/callback_queue.h"
// #include "ros/subscribe_options.h"
#include <thread>
#include <string>
#include <vector>
#include <memory>

#include <boost/bind.hpp>
#include <boost/format.hpp>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>

#include <std_msgs/msg/string.hpp>
#include <pedsim_msgs/msg/tracked_persons.hpp>
#include <pedsim_msgs/msg/agent_states.hpp>


namespace gazebo
{
    class ActorPosesPlugin : public WorldPlugin{
        public:
            ActorPosesPlugin() : WorldPlugin(){

            }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){
            std::cout << "Gazebo Load --- Start ---" << std::endl;

            // Initialize ROS node
            this->node = gazebo_ros::Node::Get(_sdf);

            // Get QoS profiles
            const gazebo_ros::QoS & qos = this->node->get_qos();

            if (!rclcpp::ok()){
                RCLCPP_ERROR(node->get_logger(), "ROS not initialized");
                return;
            }

            this->world_ = _world;
            // rosNode.reset(new ros::NodeHandle("gazebo_client"));
            
            // ros::SubscribeOptions so = ros::SubscribeOptions::create<pedsim_msgs::AgentStates>(
            //     "/pedsim_simulator/simulated_agents",
            //     1,
            //     boost::bind(&ActorPosesPlugin::OnRosMsg, this, _1),
            //     ros::VoidPtr(),&rosQueue
            // );
            // rosSub = rosNode->subscribe(so);
            // rosQueueThread =std::thread(std::bind(&ActorPosesPlugin::QueueThread, this));
            this->agent_states_sub_ = this->node->create_subscription<pedsim_msgs::msg::AgentStates>(
                "/pedsim_simulator/simulated_agents",
                qos.get_subscription_qos("/pedsim_simulator/simulated_agents", rclcpp::QoS(1)),
                std::bind(&ActorPosesPlugin::OnRosMsg, this, std::placeholders::_1)
            );

            this->test_sub_ = this->node->create_subscription<std_msgs::msg::String>(
                "/test",
                qos.get_subscription_qos("/test", rclcpp::QoS(1)),
                std::bind(&ActorPosesPlugin::OnTestMsg, this, std::placeholders::_1)
            );

            std::cout << "Gazebo Load ---  End  ---" << std::endl;
            // in case you need to change/modify model on update
            // this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&ActorPosesPlugin::OnUpdate, this));
        }


        public:
            void OnTestMsg(const std_msgs::msg::String::SharedPtr msg){
                std::lock_guard<std::mutex> scoped_lock(lock_);
                std::cout << msg->data << std::endl;
            }

            // call back function when receive rosmsg
            void OnRosMsg( const pedsim_msgs::msg::AgentStates::SharedPtr msg) {
                std::lock_guard<std::mutex> scoped_lock(lock_);

                // std::cout << "OnRosMsg --- Start ---" << std::endl;
//              ROS_INFO ("OnRosMsg ... ");
                std::string model_name;
#if GAZEBO_MAJOR_VERSION < 9
                for(unsigned int mdl = 0; mdl < world_->GetModelCount(); mdl++) {
#else
                for(unsigned int mdl = 0; mdl < world_->ModelCount(); mdl++) {
#endif
                    physics::ModelPtr  tmp_model;
#if GAZEBO_MAJOR_VERSION < 9
                    tmp_model = world_->GetModel(mdl);
#else
                    tmp_model = world_->ModelByIndex(mdl);
#endif
                    std::string frame_id;
                    frame_id = tmp_model->GetName();

                    for (uint actor =0; actor< msg->agent_states.size() ; actor++) {
                        if(frame_id == std::to_string( msg->agent_states[actor].id)  ){
//                            ROS_INFO_STREAM("actor_id: "<< std::to_string( msg->tracks[actor].track_id) );
                            ignition::math::Pose3d gzb_pose;
                            gzb_pose.Pos().Set( msg->agent_states[actor].pose.position.x,
                                                msg->agent_states[actor].pose.position.y,
                                                msg->agent_states[actor].pose.position.z + MODEL_OFFSET);
                            gzb_pose.Rot().Set(msg->agent_states[actor].pose.orientation.w,
                                               msg->agent_states[actor].pose.orientation.x,
                                               msg->agent_states[actor].pose.orientation.y,
                                               msg->agent_states[actor].pose.orientation.z);

                            try{
                                tmp_model->SetWorldPose(gzb_pose);
                                
                                // set model velocity to 0
                                ignition::math::Vector3d vel(0,0,0);
                                tmp_model->SetLinearVel(vel);
                                tmp_model->SetAngularVel(vel);
                            }
                            catch(gazebo::common::Exception gz_ex){
                                RCLCPP_ERROR(node->get_logger(), "Error setting pose");// boost::format("Error setting pose %s - %s") % frame_id.c_str() % gz_ex.GetErrorStr());
                            }

                        }
                    }
               }

          }


        // ROS helper function that processes messages
        // private: void QueueThread() {
        //     static const double timeout = 0.1;
        //     while (rosNode->ok()) {
        //         rosQueue.callAvailable(ros::WallDuration(timeout));
        //     }
        // }

    private:
        // std::unique_ptr<ros::NodeHandle> rosNode;
        gazebo_ros::Node::SharedPtr node;

        // ros::Subscriber rosSub;
        // ros::CallbackQueue rosQueue;
        // std::thread rosQueueThread;
        rclcpp::Subscription<pedsim_msgs::msg::AgentStates>::SharedPtr agent_states_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr test_sub_;

        physics::WorldPtr world_;
        event::ConnectionPtr updateConnection_;
        const float MODEL_OFFSET = 0.75;

        std::mutex lock_;

    };
    GZ_REGISTER_WORLD_PLUGIN(ActorPosesPlugin)
}


