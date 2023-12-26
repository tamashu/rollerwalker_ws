#include "rollerwalker_sim/fileWriter.hpp"

#include <cmath>

fileWriter::fileWriter(std::string file_name):nh_("~"),file_handler_(file_name){
    nh_.getParam("/file_write_frequency", file_write_frequency_);
    start_time_ = ros::Time::now();

    true_position_with_t_.resize(4);
    true_velocity_with_t_.resize(2);
    true_yaw_velocity_with_t_.resize(2);
    
    true_position_sub_      = nh_.subscribe("/true_position"      , 1, &fileWriter::truePositionCallback_, this);
    true_velocity_sub_      = nh_.subscribe("/true_velocity"      , 1, &fileWriter::trueVelocityCallback_, this);
    true_yaw_velocity_sub_  = nh_.subscribe("/true_yaw_velocity"  , 1, &fileWriter::trueYawVelocityCallback_, this);

    timer_ =nh_.createTimer(ros::Duration(1/(double)file_write_frequency_), &fileWriter::timerFileWriteCallback_, this);
}
void fileWriter::trueVelocityCallback_(const std_msgs::Float64& msg){
    double true_velocity = msg.data;
    ros::Duration duration = (ros::Time::now() - start_time_);
    double time = duration.sec + duration.nsec/pow(10,9);

    true_velocity_with_t_[0] = time;
    true_velocity_with_t_[1] = true_velocity;
}

void fileWriter::truePositionCallback_(const nav_msgs::Odometry& msg){
    ros::Duration duration = (ros::Time::now() - start_time_);
    double time = duration.sec + duration.nsec/pow(10,9);

    true_position_with_t_[0] = time;
    true_position_with_t_[1] = msg.pose.pose.position.x;
    true_position_with_t_[2] = msg.pose.pose.position.y;
    true_position_with_t_[3] = msg.pose.pose.position.z;
}

void fileWriter::trueYawVelocityCallback_(const std_msgs::Float64& msg){
    double true_yaw_velocity = msg.data;

    ros::Duration duration = (ros::Time::now() - start_time_);
    double time = duration.sec + duration.nsec/pow(10,9);

    true_yaw_velocity_with_t_[0] = time;
    true_yaw_velocity_with_t_[1] = true_yaw_velocity;
}

void fileWriter::timerFileWriteCallback_(const ros::TimerEvent& e){
    file_handler_.writeFile(true_position_with_t_);
}

int main(int argc, char** argv)
{
    std::string file_name = "/home/lab/rollerwalker_ws/src/rollerwalker_sim/data/true_velocity.csv";
    ros::init(argc, argv, "file_writer");

    fileWriter file_writer(file_name);
    ros::spin();
    return 0;
}


