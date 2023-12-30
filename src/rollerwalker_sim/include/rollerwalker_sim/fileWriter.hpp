#pragma once
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <rollerwalker_msgs/Velocity.h>
#include <rollerwalker_msgs/Position.h>

#include "rollerwalker_sim/file_handler.hpp"

class fileWriter{
public:
    fileWriter(std::string position_file_name,std::string velocity_file_name);
    void truePositionCallback_(const rollerwalker_msgs::Position& msg);
    void trueVelocityCallback_(const rollerwalker_msgs::Velocity& msg);
    void timerFileWriteCallback_(const ros::TimerEvent& e);
    void isStartCallback_(const std_msgs::Bool& msg);
    
private:
    int file_write_frequency_;      //ファイル出力の周期
    bool is_start_flag_ = false;    //キーボードからの入力

    fileHandler position_file_handler_;
    fileHandler velocity_file_handler_;

    std::vector<double> true_position_with_t_;          //[0]:t [1]:x [2]:y [3]:z
    std::vector<double> true_velocity_with_t_;          //[0]:t [1]:v [2]yaw_v

    ros::NodeHandle nh_;
    ros::Timer timer_;
    ros::Time start_time_;  //rosの開始時間
    ros::Subscriber is_start_flag_sub_; //is_start_flagのサブスクライバ
    ros::Subscriber true_position_sub_, true_velocity_sub_; //ture_position,true_velocityのサブスクライバ
};