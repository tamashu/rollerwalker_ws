#include "rollerwalker_sim/fileWriter.hpp"

#include <cmath>

fileWriter::fileWriter(std::string position_file_name,std::string velocity_file_name)
    :nh_("~"),position_file_handler_(position_file_name),velocity_file_handler_(velocity_file_name) 
{
    nh_.getParam("/file_write_frequency", file_write_frequency_);
    start_time_ = ros::Time::now();

    true_position_with_t_.resize(4);
    true_velocity_with_t_.resize(3);
    
    true_position_sub_      = nh_.subscribe("/true_position_x_y_z"      , 1, &fileWriter::truePositionCallback_, this);
    true_velocity_sub_      = nh_.subscribe("/true_velocity"      , 1, &fileWriter::trueVelocityCallback_, this);

    timer_ =nh_.createTimer(ros::Duration(1/(double)file_write_frequency_), &fileWriter::timerFileWriteCallback_, this);
}
void fileWriter::trueVelocityCallback_(const rollerwalker_msgs::Velocity& msg){
    double velocity_x = msg.velocity_x;
    double velocity_y = msg.velocity_y;

    float V = sqrt(pow(velocity_x,2)+pow(velocity_y,2));

    true_velocity_with_t_[0] = msg.t;
    true_velocity_with_t_[1] = V;
    true_velocity_with_t_[2] = msg.yaw_velocity;
}

void fileWriter::truePositionCallback_(const rollerwalker_msgs::Position& msg){
    true_position_with_t_[0] = msg.t;
    true_position_with_t_[1] = msg.x;
    true_position_with_t_[2] = msg.y;
    true_position_with_t_[3] = msg.z;
}


void fileWriter::timerFileWriteCallback_(const ros::TimerEvent& e){
    position_file_handler_.writeFile(true_position_with_t_);
    velocity_file_handler_.writeFile(true_velocity_with_t_);
}

int main(int argc, char** argv)
{   
    std::string position_file_name;
    std::string velocity_file_name;    

    ros::init(argc, argv, "file_writer");
    ros::NodeHandle nh;
    nh.getParam("/position_file_name", position_file_name);
    nh.getParam("/velocity_file_name", velocity_file_name);

    fileWriter file_writer(position_file_name,velocity_file_name);
    ros::spin();
    return 0;
}


