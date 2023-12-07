#include "rollerwalker_sim/RollerwalkerDriver.hpp"

#include <string>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define PI 3.1415926535897932384626433832795

RollerwalkerDriver::RollerwalkerDriver(double d_0, double theta_0, double omega, double phi,  double phi_fr, double center_z, bool is_rollerwalk)
        : BaseRollerwalker(d_0, theta_0, omega, phi, phi_fr, center_z , is_rollerwalk),nh_("~")
{
    //トピック名の設定
    for(int i=0;i<rollerwalker_joints_pub_lf_.size();i++){   //lf
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint_" << i + 1 << "_lf_trans/command";
        rollerwalker_joints_pub_lf_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
    for(int i=0;i<rollerwalker_joints_pub_lr_.size();i++){   //lr
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint_" << i + 1 << "_lr_trans/command";
        rollerwalker_joints_pub_lr_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
    for(int i=0;i<rollerwalker_joints_pub_rr_.size();i++){   //rr
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint_" << i + 1 << "_rr_trans/command";
        rollerwalker_joints_pub_rr_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
    for(int i=0;i<rollerwalker_joints_pub_rf_.size();i++){   //rf
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint_" << i + 1 << "_rf_trans/command";
        rollerwalker_joints_pub_rf_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }

    //flagの設定
    is_changing_mode_ = false;
    is_start_flag_ = false;
    is_rollerwalk_ = is_rollerwalk;
    is_complete_mode_change_ = true;
    is_start_flag_sub_ = nh_.subscribe("/is_start_flag", 2, &RollerwalkerDriver::isStartCallback_, this);
    is_rollerwalk_flag_sub_ = nh_.subscribe("/is_rollerwalk_flag", 2, &RollerwalkerDriver::isRollerwalkCallback_, this);

    //jointの状態を受け取るサブスクライバの設定
    joint_positions_lf_sub_ = nh_.subscribe("/joint_positions_lf", 5, &RollerwalkerDriver::jointPositionLFCallback_, this);
    joint_positions_lr_sub_ = nh_.subscribe("/joint_positions_lr", 5, &RollerwalkerDriver::jointPositionLRCallback_, this);
    joint_positions_rr_sub_ = nh_.subscribe("/joint_positions_rr", 5, &RollerwalkerDriver::jointPositionRRCallback_, this);
    joint_positions_rf_sub_ = nh_.subscribe("/joint_positions_rf", 5, &RollerwalkerDriver::jointPositionRFCallback_, this);

    
}

//flag_callback
void RollerwalkerDriver::isStartCallback_(const std_msgs::Bool& msg)
{
  is_start_flag_ = msg.data;
}
void RollerwalkerDriver::isRollerwalkCallback_(const std_msgs::Bool& msg)
{
  is_rollerwalk_ = msg.data;
  if(! is_rollerwalk_){
    changeToWalk();
  }
}

//joint_position_callback
void RollerwalkerDriver::jointPositionLFCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        
        // ROS_INFO("[%i]:%f", i, msg.data[i]);
        joint_position_lf_[i] = msg.data[i];
    }
}
void RollerwalkerDriver::jointPositionLRCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        joint_position_lr_[i] = msg.data[i];
    }
}
void RollerwalkerDriver::jointPositionRRCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        joint_position_rr_[i] = msg.data[i];
    }
}
void RollerwalkerDriver::jointPositionRFCallback_(const std_msgs::Float32MultiArray& msg){
    for(int i =0;i<msg.data.size();i++){
        joint_position_rf_[i] = msg.data[i];
    }
}

bool RollerwalkerDriver::getIsStartFlag_(){
    return is_start_flag_;
}
bool RollerwalkerDriver::getIsRollerwalk(){
    return is_rollerwalk_;
}
bool RollerwalkerDriver::getChangingFlag(){
    return is_changing_mode_;
}

void RollerwalkerDriver::changeToWalk(){
    is_changing_mode_ = true;
    is_start_flag_ = false;
    is_complete_mode_change_ = false;

    std_msgs::Float64 joint1_msg;
    std_msgs::Float64 joint2_msg;
    std_msgs::Float64 joint3_msg;
    std_msgs::Float64 joint4_msg;
    
    // 脚をt=n*ωの位置（初期位置）に戻す
    double omega = getOmega();
    double r = fmod(t_ , omega);  
    double n = (t_-r)/omega;
    ROS_INFO("omega: %f  t_: %f  r: %f  n: %f",omega,t_,r,n);
    while(omega*n<t_){
        t_ -=0.01;
        jointsPublish_(t_);
        sleep(0.1);
        //ROS_INFO("t_: %f",t_);
    }
    sleep(1.0);
    //jointsPublish_(0.0);
    // while(joint_position_lf_[1]<joint_2_limit){
    //     joint2_msg.data =joint_2_limit + joint_position_margine;    
    //     rollerwalker_joints_pub_lf_[1].publish(joint2_msg);
    //     joint3_msg.data =joint_3_limit + joint_position_margine;    
    //     rollerwalker_joints_pub_lf_[2].publish(joint3_msg);
    //     //ROS_INFO("joint_position_lf[1]:%f  joint_position_lf[2]:%f",joint_position_lf_[1],joint_position_lf_[2]);
    // }
    ROS_INFO("out joint_position_lf[1]:%f",joint_position_lf_[1]);
    is_complete_mode_change_ = true;
    is_changing_mode_ = false;
}

void RollerwalkerDriver::jointsPublish_(double t){
    t_ = t;
    calAndSetTheta(t);
    std_msgs::Float64 joint1_msg;
    std_msgs::Float64 joint2_msg;
    std_msgs::Float64 joint3_msg;
    std_msgs::Float64 joint4_msg;
    //lf
    joint1_msg.data =getTheta1LF();
    joint2_msg.data =getTheta2LF();
    joint3_msg.data =getTheta3LF();
    joint4_msg.data =getTheta4LF();

    rollerwalker_joints_pub_lf_[0].publish(joint1_msg);
    rollerwalker_joints_pub_lf_[1].publish(joint2_msg);
    rollerwalker_joints_pub_lf_[2].publish(joint3_msg);
    rollerwalker_joints_pub_lf_[3].publish(joint4_msg);
    //lr
    joint1_msg.data =getTheta1LR();
    joint2_msg.data =getTheta2LR();
    joint3_msg.data =getTheta3LR();
    joint4_msg.data =getTheta4LR();

    rollerwalker_joints_pub_lr_[0].publish(joint1_msg);
    rollerwalker_joints_pub_lr_[1].publish(joint2_msg);
    rollerwalker_joints_pub_lr_[2].publish(joint3_msg);
    rollerwalker_joints_pub_lr_[3].publish(joint4_msg);
    //rr
    joint1_msg.data =getTheta1RR();
    joint2_msg.data =getTheta2RR();
    joint3_msg.data =getTheta3RR();
    joint4_msg.data =getTheta4RR();

    rollerwalker_joints_pub_rr_[0].publish(joint1_msg);
    rollerwalker_joints_pub_rr_[1].publish(joint2_msg);
    rollerwalker_joints_pub_rr_[2].publish(joint3_msg);
    rollerwalker_joints_pub_rr_[3].publish(joint4_msg);
    //rf
    joint1_msg.data =getTheta1RF();
    joint2_msg.data =getTheta2RF();
    joint3_msg.data =getTheta3RF();
    joint4_msg.data =getTheta4RF();

    rollerwalker_joints_pub_rf_[0].publish(joint1_msg);
    rollerwalker_joints_pub_rf_[1].publish(joint2_msg);
    rollerwalker_joints_pub_rf_[2].publish(joint3_msg);
    rollerwalker_joints_pub_rf_[3].publish(joint4_msg);
}



int main(int argc, char** argv)
{
    // ローラウォーカーの各ステータス
    double d_0 = 0.08;
	double theta_0 = 0.15;
	double omega = PI;
	double phi = PI / 2;
	//double phi_fr = 3 * PI / 2;
    double phi_fr = 0;
    double  center_z = 0.15;      
    bool is_rollerWalk = true;

    ros::init(argc, argv, "rollerwalker_driver");
    RollerwalkerDriver rollerwalker_driver(d_0, theta_0, omega, phi, phi_fr, center_z , is_rollerWalk);
    ros::Rate loop_rate(10);
    int count = 0;
    double t =0.0;
    while (ros::ok())
    {   
        if(rollerwalker_driver.getChangingFlag()){
            count = 0;
        }

        // if(! rollerwalker_driver.is_rollerwalk_){   //歩行モードフラグの時
        //     if(! rollerwalker_driver.is_changing_mode_ && ! rollerwalker_driver.is_complete_mode_change_){ //モードが変更中でなく、モード変更が完了していない時
        //         rollerwalker_driver.changeToWalk();
        //     }
        // }

        if(rollerwalker_driver.getIsStartFlag_()){
                count++;
                t = (double)count/10.0;
        }
        if(rollerwalker_driver.getIsRollerwalk()){
                rollerwalker_driver.jointsPublish_(t);
        }
        
        // ROS_INFO("time: %f",t);
        
        //ROS_INFO("time:t: %.2f  ",t );
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}