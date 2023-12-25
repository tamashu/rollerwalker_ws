#include "rollerwalker_sim/RollerwalkerDriver.hpp"

#include <string>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>


#define PI 3.1415926535897932384626433832795

RollerwalkerDriver::RollerwalkerDriver(double phi, double l2, double l3, double l4, double wheel_radius, double wheel_thickenss)
        : BaseRollerwalker(phi,l2,l3,l4,wheel_radius,wheel_thickenss),nh_("~")
{

    // 初期化
    nh_.getParam("/rollerwalker_param/d_0", d_0_);
    nh_.getParam("/rollerwalker_param/theta_0_lf", theta_0_lf_);
    nh_.getParam("/rollerwalker_param/theta_0_lr", theta_0_lr_);
    nh_.getParam("/rollerwalker_param/theta_0_rr", theta_0_rr_);
    nh_.getParam("/rollerwalker_param/theta_0_rf", theta_0_rf_);
    nh_.getParam("/rollerwalker_param/omega", omega_);
    nh_.getParam("/rollerwalker_param/phi_fr", phi_fr_);
    nh_.getParam("/rollerwalker_param/center_z", center_z_);

    phi_ = phi;

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
    is_rollerwalk_ = true;
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

std::array<double, 4> RollerwalkerDriver::calAllTheta(double t, double d_0, double theta_0, double steering_ofset, double omega, double phi_fr, double center_z){
    std::array<double, 4> ret;
    double d = calD(t,d_0,omega,phi_fr);

    double theta_1 = calTheta(t,theta_0,omega,phi_fr,steering_ofset);
	double theta_2 = calTheta2(d,center_z, is_rollerwalk_);
	double theta_3 = calTheta3(d,theta_2,center_z, is_rollerwalk_) + PI/2;
	double theta_4 = calTheta4(theta_2,theta_3,is_rollerwalk_);

    ret[0] = theta_1;
    ret[1] = theta_2;
    ret[2] = theta_3;
    ret[3] = theta_4;

    return ret; 
};

void RollerwalkerDriver::currentCenterZCallback_(const std_msgs::Float64& msg){

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

    std::array<double ,4> theta_lf = calAllTheta(t,d_0_,theta_0_lf_,0,omega_,0,center_z_);
    std::array<double ,4> theta_lr = calAllTheta(t,d_0_,theta_0_lr_,0,omega_,phi_fr_,center_z_);
    std::array<double ,4> theta_rr = calAllTheta(t,d_0_,theta_0_rr_,0,omega_,phi_fr_,center_z_);
    std::array<double ,4> theta_rf = calAllTheta(t,d_0_,theta_0_rf_,0,omega_,0,center_z_);

    std::array<std_msgs::Float64,4> joint_msgs_lf;
    std::array<std_msgs::Float64,4> joint_msgs_lr;
    std::array<std_msgs::Float64,4> joint_msgs_rr;
    std::array<std_msgs::Float64,4> joint_msgs_rf;

    for(int i=0;i<4;i++){
        joint_msgs_lf[i].data = theta_lf[i];
        joint_msgs_lr[i].data = theta_lr[i];
        joint_msgs_rr[i].data = theta_rr[i];
        joint_msgs_rf[i].data = theta_rf[i];

        rollerwalker_joints_pub_lf_[i].publish(joint_msgs_lf[i]);
        rollerwalker_joints_pub_lr_[i].publish(joint_msgs_lr[i]);
        rollerwalker_joints_pub_rr_[i].publish(joint_msgs_rr[i]);
        rollerwalker_joints_pub_rf_[i].publish(joint_msgs_rf[i]);
    }
}

int main(int argc, char** argv)
{
    double phi;
    double l2;
    double l3;
    double l4;
    double wheel_radius;
    double wheel_thickness;

    int publish_frequency;    //Hz

    ros::init(argc, argv, "rollerwalker_driver");
    ros::NodeHandle nh;     //rosparam用のノードハンドラ

    nh.getParam("/rollerwalker_param/phi", phi);
    nh.getParam("/rollerwalker_param/l2", l2);
    nh.getParam("/rollerwalker_param/l3", l3);
    nh.getParam("/rollerwalker_param/l4", l4);
    nh.getParam("/rollerwalker_param/wheel_radius", wheel_radius);
    nh.getParam("/rollerwalker_param/wheel_thicness", wheel_thickness);

    
    nh.getParam("/publish_frequency", publish_frequency);
    

    RollerwalkerDriver rollerwalker_driver(phi,l2,l3,l4,wheel_radius,wheel_thickness);

    ros::Rate loop_rate(publish_frequency);
    int count = 0;
    double t =0.0;
    while (ros::ok())
    {   
        if(rollerwalker_driver.getChangingFlag()){
            count = 0;
        }
        if(rollerwalker_driver.getIsStartFlag_()){
                count++;
                t = (double)count/publish_frequency;
        }
        if(rollerwalker_driver.getIsRollerwalk()){
                rollerwalker_driver.jointsPublish_(t);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}