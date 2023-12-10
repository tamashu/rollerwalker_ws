#include "rollerwalker_sim/WheelOdomPub.hpp"
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

WheelOdomPub::WheelOdomPub():nh_("~")
{
    nh_.getParam("/rollerwalker_param/l2", l2_);
    nh_.getParam("/rollerwalker_param/l3", l3_);
    nh_.getParam("/rollerwalker_param/l4", l4_);
    nh_.getParam("/rollerwalker_param/wheel_radius", wheel_radius_);
    nh_.getParam("/rollerwalker_param/body_width", body_width_);

    //jointの状態を受け取るサブスクライバの設定
    joint_positions_lf_sub_ = nh_.subscribe("/joint_positions_lf", 5, &WheelOdomPub::jointPositionLFCallback_, this);
    joint_positions_lr_sub_ = nh_.subscribe("/joint_positions_lr", 5, &WheelOdomPub::jointPositionLRCallback_, this);
    joint_positions_rr_sub_ = nh_.subscribe("/joint_positions_rr", 5, &WheelOdomPub::jointPositionRRCallback_, this);
    joint_positions_rf_sub_ = nh_.subscribe("/joint_positions_rf", 5, &WheelOdomPub::jointPositionRFCallback_, this);
    wheel_velocity_sub_     = nh_.subscribe("/wheel_velocity"    , 5, &WheelOdomPub::wheelVelocityCallback_, this);

    //publidherの設定
    velocity_pub_ = nh_.advertise<std_msgs::Float64>("/velocity", 5);

    //timercallbavkの設定
    timer_ =nh_.createTimer(ros::Duration(0.1), &WheelOdomPub::timerWheelVelocityPubCallback_, this);


}

//joint_position_callback
void WheelOdomPub::jointPositionLFCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        // ROS_INFO("[%i]:%f", i, msg.data[i]);
        joint_position_lf_[i] = msg.data[i];
    }
    d_[0] = calD_(joint_position_lf_[1],joint_position_lf_[2],joint_position_lf_[3]);
}
void WheelOdomPub::jointPositionLRCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        joint_position_lr_[i] = msg.data[i];
    }
    d_[1] = calD_(joint_position_lr_[1],joint_position_lr_[2],joint_position_lr_[3]);

}
void WheelOdomPub::jointPositionRRCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        joint_position_rr_[i] = msg.data[i];
    }
    d_[2] = calD_(joint_position_rr_[1],joint_position_rr_[2],joint_position_rr_[3]);
}
void WheelOdomPub::jointPositionRFCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        joint_position_rf_[i] = msg.data[i];
    }
    d_[3] = calD_(joint_position_rf_[1],joint_position_rf_[2],joint_position_rf_[3]);
}
void WheelOdomPub::wheelVelocityCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        // wheel_velocity_[i] = msg.data[i];
        wheel_velocity_[i] = calMovingAverage(msg.data[i],filtering_wheel_velocity_[i],wheel_counters_[i],wheel_sums_[i]) ;    
    }

    // ROS_INFO("lf:%8.5f  lr:%8.5f  rr:%8.5f  rf:%8.5f",wheel_velocity_[0],wheel_velocity_[1],wheel_velocity_[2],wheel_velocity_[3]);
}
//odometryのパブリッシュ
void WheelOdomPub::timerWheelVelocityPubCallback_(const ros::TimerEvent& e){
    std_msgs::Float64 velocity_msg;
    double velocity_left = wheel_radius_*calAverage(wheel_velocity_[0]*cos(joint_position_lf_[0]),wheel_velocity_[1*cos(joint_position_lr_[0])]);       //左のタイヤの速度（平均）
    double velocity_right = -wheel_radius_ * calAverage(wheel_velocity_[2]*cos(joint_position_rr_[0]),wheel_velocity_[3]*cos(joint_position_rf_[0]));     //右のタイヤの速度（平均）（符号はマイナス）

    ROS_INFO("V_left: %f  V_right: %f ",velocity_left,velocity_right);

    velocity_msg.data =calAverage(velocity_left , velocity_right);
    velocity_pub_.publish(velocity_msg);
};

double WheelOdomPub::calD_(double theta_2,double theta_3,double theta_4){
    double x_e = l2_ * cos(theta_2) + l3_ * sin(theta_2 + theta_3) + l4_ * sin(theta_2 + theta_3 + theta_4);
    return x_e;
}
double WheelOdomPub::calZ_(double theta_2,double theta_3,double theta_4){
    double z_e = l2_ * sin(theta_2) + l3_ * cos(theta_2 + theta_3) + l4_ * cos(theta_2 + theta_3 + theta_4);
    return z_e;
}
double WheelOdomPub::calAverage(double input1, double input2){
    double ret = (input1+input2)/2;
    return ret;
}
double WheelOdomPub::calMovingAverage(double input_value, std::array<float,NUM_OF_FILTERING>filterring_array, int& count, float&  sum){
    if (count == NUM_OF_FILTERING){
        count = 0;
    } 
    sum -= filterring_array[count];
    filterring_array[count] = input_value;
    sum += filterring_array[count];
    count++;
    double average = sum / NUM_OF_FILTERING;
    return average;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odom_pub");
    WheelOdomPub wheel_odom_pub;
    ros::spin();
    return 0;
}