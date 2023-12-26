#pragma once
#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include "rollerwalker_sim/file_handler.hpp"

class fileWriter{
public:
    fileWriter(std::string file_name);
    void truePositionCallback_(const nav_msgs::Odometry& msg);
    void trueVelocityCallback_(const std_msgs::Float64& msg);
    void trueYawVelocityCallback_(const std_msgs::Float64& msg);
    void timerFileWriteCallback_(const ros::TimerEvent& e);
    
private:
    int file_write_frequency_;
    fileHandler file_handler_;

    std::vector<double> true_position_with_t_;          //[0]:t [1]:x [2]:y [3]:z
    std::vector<double> true_velocity_with_t_;          //[0]:t [1]:v
    std::vector<double> true_yaw_velocity_with_t_;      //[0]:t [1]:yaw_v

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Time start_time_;  //rosの開始時間
    ros::Subscriber true_position_sub_, true_velocity_sub_, true_yaw_velocity_sub_;
};