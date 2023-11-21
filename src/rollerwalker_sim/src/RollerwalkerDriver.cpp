#include "rollerwalker_sim/RollerwalkerDriver.hpp"

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>


RollerwalkerDriver::RollerwalkerDriver(double d_0, double theta_0, double omega, double phi,  double phi_fr, double center_z, bool is_rollerWalk)
        : BaseRollerwalker(d_0, theta_0, omega, phi, phi_fr, center_z , is_rollerWalk),nh_("~")
{
    //トピック名の設定
    for(int i=0;i<rollerwalker_joints_pub_lf_.size();i++){   //lf
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint" << i + 1 << "lf_trans/command";
        rollerwalker_joints_pub_lf_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
    for(int i=0;i<rollerwalker_joints_pub_lr_.size();i++){   //lr
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint" << i + 1 << "lr_trans/command";
        rollerwalker_joints_pub_lr_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
    for(int i=0;i<rollerwalker_joints_pub_rr_.size();i++){   //rr
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint" << i + 1 << "rr_trans/command";
        rollerwalker_joints_pub_rr_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
    for(int i=0;i<rollerwalker_joints_pub_rf_.size();i++){   //rf
        std::stringstream topic_name;
        topic_name << "/rollerwalker/joint" << i + 1 << "rf_trans/command";
        rollerwalker_joints_pub_rf_[i] = nh_.advertise<std_msgs::Float64>(topic_name.str(), 10);
    }
}

void RollerwalkerDriver::jointsPublish_(double t){
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

#define PI 3.1415926535897932384626433832795

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
    while (ros::ok())
    {   
        count++;
        double t = (double)count/10.0;
        ROS_INFO("time: %f",t);
        rollerwalker_driver.jointsPublish_(t);
        //ROS_INFO("time:t: %.2f  ",t );
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}