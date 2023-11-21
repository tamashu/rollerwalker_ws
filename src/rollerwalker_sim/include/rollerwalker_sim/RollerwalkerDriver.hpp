#pragma once
#include <ros/ros.h>
#include "rollerwalker_sim/BaseRollerwalker.hpp"

class RollerwalkerDriver : public BaseRollerwalker
{
public:
    RollerwalkerDriver(double d_0, double theta_0, double omega, double phi,  double phi_fr, double center_z, bool is_rollerWalk);
    void jointsPublish_(double t);
private:
    ros::NodeHandle nh_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_lf_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_lr_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_rr_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_rf_;
    
};  