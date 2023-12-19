#include "rollerwalker_sim/RollerwalkerDriver.hpp"

#include <string>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>


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

    //center_zの設定（フィードバック)
    current_center_z_sub_ = nh_.subscribe("/current_center_z", 1, &RollerwalkerDriver::currentCenterZCallback_, this);
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
    // changeToWalk();
  }
}

void RollerwalkerDriver::currentCenterZCallback_(const std_msgs::Float64& msg){
        //setFrontCenterZ(msg.data);
        // setBackCenterZ(msg.data);
        ROS_INFO("backCenter_z:%.4f",msg.data);
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

void RollerwalkerDriver::jointsPublish_(double t){
    t_ = t;

    // setSteering_ofset_lf_(0.3);
    // setSteering_ofset_lr_(-0.3);
    // setSteering_ofset_rr_(0.3);
    // setSteering_ofset_rf_(-0.3);

    //旋回
    setTheta_0_rf_(0);
    setTheta_0_rr_(0);

    calAndSetTheta(t);
    


    std_msgs::Float64 joint1_msg;
    std_msgs::Float64 joint2_msg;
    std_msgs::Float64 joint3_msg;
    std_msgs::Float64 joint4_msg;

    //lf
    joint1_msg.data =getTheta1LF_();
    joint2_msg.data =getTheta2LF_();
    joint3_msg.data =getTheta3LF_();
    joint4_msg.data =getTheta4LF_();

    rollerwalker_joints_pub_lf_[0].publish(joint1_msg);
    rollerwalker_joints_pub_lf_[1].publish(joint2_msg);
    rollerwalker_joints_pub_lf_[2].publish(joint3_msg);
    rollerwalker_joints_pub_lf_[3].publish(joint4_msg);
    //lr
    joint1_msg.data =getTheta1LR_();
    joint2_msg.data =getTheta2LR_();
    joint3_msg.data =getTheta3LR_();
    joint4_msg.data =getTheta4LR_();

    rollerwalker_joints_pub_lr_[0].publish(joint1_msg);
    rollerwalker_joints_pub_lr_[1].publish(joint2_msg);
    rollerwalker_joints_pub_lr_[2].publish(joint3_msg);
    rollerwalker_joints_pub_lr_[3].publish(joint4_msg);
    //rr
    joint1_msg.data =getTheta1RR_();
    joint2_msg.data =getTheta2RR_();
    joint3_msg.data =getTheta3RR_();
    joint4_msg.data =getTheta4RR_();

    rollerwalker_joints_pub_rr_[0].publish(joint1_msg);
    rollerwalker_joints_pub_rr_[1].publish(joint2_msg);
    rollerwalker_joints_pub_rr_[2].publish(joint3_msg);
    rollerwalker_joints_pub_rr_[3].publish(joint4_msg);
    //rf
    joint1_msg.data =getTheta1RF_();
    joint2_msg.data =getTheta2RF_();
    joint3_msg.data =getTheta3RF_();
    joint4_msg.data =getTheta4RF_();

    rollerwalker_joints_pub_rf_[0].publish(joint1_msg);
    rollerwalker_joints_pub_rf_[1].publish(joint2_msg);
    rollerwalker_joints_pub_rf_[2].publish(joint3_msg);
    rollerwalker_joints_pub_rf_[3].publish(joint4_msg);
}



int main(int argc, char** argv)
{
    // ローラウォーカーの各ステータス
    double d_0 = 0.08;
	double theta_0 = 0.3;
	double omega = PI;
	double phi = PI / 2;
	// double phi_fr = PI / 2;
    double phi_fr = 0;
    double  center_z = 0.15;      
    bool is_rollerWalk = true;

    ros::init(argc, argv, "rollerwalker_driver");
    RollerwalkerDriver rollerwalker_driver(d_0, theta_0, omega, phi, phi_fr, center_z , is_rollerWalk);
    ros::Rate loop_rate(50);
    int count = 0;
    double t =0.0;
    while (ros::ok())
    {   
        if(rollerwalker_driver.getChangingFlag()){
            count = 0;
        }
        if(rollerwalker_driver.getIsStartFlag_()){
                count++;
                t = (double)count/50.0;
        }
        if(rollerwalker_driver.getIsRollerwalk()){
                rollerwalker_driver.jointsPublish_(t);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}