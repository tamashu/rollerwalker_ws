#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>


class JointStateReciever
{
public:
    JointStateReciever();
    void jointSubCalback_(const sensor_msgs::JointState::ConstPtr& msg);
    void timerJointPositionPublishCalback(const ros::TimerEvent& e);
private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub_;
    ros::Publisher joint_positions_lf_pub_,joint_positions_lr_pub_,joint_positions_rr_pub_,joint_positions_rf_pub_;
    ros::Timer timer_;

    std::array<float, 4> joint_lf_positions_{}; // rad
    std::array<float, 4> joint_lr_positions_{}; // rad
    std::array<float, 4> joint_rr_positions_{}; // rad
    std::array<float, 4> joint_rf_positions_{}; // rad
    std::array<float, 4> wheel_velocity_{}; // rad/s
    std::array<std::array<float, 4>,4> joint_positions_{joint_lf_positions_,joint_lr_positions_,joint_rr_positions_,joint_rf_positions_};

    std::array<ros::Publisher, 4> joint_positions_pubs_;
    ros::Publisher wheel_velocity_pub_;


};