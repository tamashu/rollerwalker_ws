#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "rollerwalker_sim/BaseRollerwalker.hpp"


class RollerwalkerDriver : public BaseRollerwalker
{
public:
    RollerwalkerDriver(double d_0, double theta_0, double omega, double phi,  double phi_fr, double center_z, bool is_rollerWalk);
    void jointsPublish_(double t);
    void isStartCallback_(const std_msgs::Bool& msg);
    bool getIsStartFlag_();
private:
    ros::NodeHandle nh_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_lf_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_lr_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_rr_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_rf_;

    //subscriber
    bool is_start_flag_; //trueの時シミュレーションが動く
    ros::Subscriber is_start_flag_sub_;
    
};  