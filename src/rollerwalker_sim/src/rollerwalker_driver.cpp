#include "rollerwalker_sim/BaseRollerwalker.hpp"

#include <string>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>




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


    BaseRollerWalker base_rollerWalker(d_0, theta_0, omega, phi, phi_fr, center_z , is_rollerWalk);

    ros::init(argc, argv, "vis_joint_publisher");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {   
        double t = (double)count/10.0;
        sensor_msgs::JointState js0;
        js0.header.stamp = ros::Time::now();
        js0.name.resize(20);
        // 左前脚の設定
        js0.name[0] = "body_to_joint_1_lf";
        js0.name[1] = "joint_1_lf_to_joint_2_lf";
        js0.name[2] = "link_2_lf_to_joint_3_lf";
        js0.name[3] = "link_3_lf_to_joint_4_lf";
        js0.name[4] = "link_4_lf_to_wheel_lf";
        // 左後ろ脚の設定
        js0.name[5] = "body_to_joint_1_lr";
        js0.name[6] = "joint_1_lr_to_joint_2_lr";
        js0.name[7] = "link_2_lr_to_joint_3_lr";
        js0.name[8] = "link_3_lr_to_joint_4_lr";
        js0.name[9] = "link_4_lr_to_wheel_lr";
        // 右後ろ脚の設定
        js0.name[10] = "body_to_joint_1_rr";
        js0.name[11] = "joint_1_rr_to_joint_2_rr";
        js0.name[12] = "link_2_rr_to_joint_3_rr";
        js0.name[13] = "link_3_rr_to_joint_4_rr";
        js0.name[14] = "link_4_rr_to_wheel_rr";
        // 右前脚の設定
        js0.name[15] = "body_to_joint_1_rf";
        js0.name[16] = "joint_1_rf_to_joint_2_rf";
        js0.name[17] = "link_2_rf_to_joint_3_rf";
        js0.name[18] = "link_3_rf_to_joint_4_rf";
        js0.name[19] = "link_4_rf_to_wheel_rf";

        
        
        
        //各関節の値のセット
         base_rollerWalker.calAndSetTheta(t);

        js0.position.resize(20);
        //左前脚
        js0.position[0] = base_rollerWalker.getTheta1LF();    //第一関節
        js0.position[1] = base_rollerWalker.getTheta2LF();    //第二関節
        js0.position[2] = base_rollerWalker.getTheta3LF();    //第三関節
        js0.position[3] = base_rollerWalker.getTheta4LF();    //第四関節
        js0.position[4] = t;    //ホイール
        //左後ろ脚
        js0.position[5] = base_rollerWalker.getTheta1LR();    //第一関節
        js0.position[6] = base_rollerWalker.getTheta2LR();      //第二関節
        js0.position[7] = base_rollerWalker.getTheta3LR();      //第三関節
        js0.position[8] = base_rollerWalker.getTheta4LR();      //第四関節
        js0.position[9] = t;      //ホイール
        // //右後ろ脚
        js0.position[10] = base_rollerWalker.getTheta1RR();   //第一関節
        js0.position[11] = base_rollerWalker.getTheta2RR();     //第二関節
        js0.position[12] = base_rollerWalker.getTheta3RR();     //第三関節
        js0.position[13] = base_rollerWalker.getTheta4RR();     //第四関節
        js0.position[14] = t;     //ホイール
        //右前脚
        js0.position[15] = base_rollerWalker.getTheta1RF();   //第一関節
        js0.position[16] = base_rollerWalker.getTheta2RF();     //第二関節
        js0.position[17] = base_rollerWalker.getTheta3RF();     //第三関節
        js0.position[18] = base_rollerWalker.getTheta4RF();     //第四関節
        js0.position[19] = t;     //ホイール
        joint_pub.publish(js0);
        count++;

        //ROS_INFO("time:t: %.2f  ",t );
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}