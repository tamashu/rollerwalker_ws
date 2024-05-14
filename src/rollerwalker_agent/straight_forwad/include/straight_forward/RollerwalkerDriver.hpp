#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <array>
#include "straight_forward/BaseRollerwalker.hpp"
#include <std_msgs/Float64.h>


class RollerwalkerDriver : public BaseRollerwalker
{
public:
    RollerwalkerDriver(double phi, double l2, double l3, double l4, double wheel_radius, double wheel_thickenss);
    void jointsPublish_(double t);
    void changeToWalk();

    //callback関数
    //flag関連
    void isStartCallback_(const std_msgs::Bool& msg);
    void isRollerwalkCallback_(const std_msgs::Bool& msg);
    void currentCenterZCallback_(const std_msgs::Float64& msg);
    //joint_positon関連
    bool getIsStartFlag_();
    bool getIsRollerwalk();
    bool getChangingFlag();
private:
    std::array<double, 4> calAllTheta(double t, double d_0, double theta_0, double steering_ofset, double omega, double phi_fr, double center_z);
    const double PI =3.1415926535897932384626433832795;

    //ローラーウォーカーの4つのパラメータ
    double d_0_;            //法線方向の振幅
	double theta_0_lf_;     //接線方向の振幅(左前)
    double theta_0_lr_;     //接線方向の振幅(左後ろ)         
    double theta_0_rr_;     //接線方向の振幅(右後ろ) 
    double theta_0_rf_;     //接線方向の振幅(右前) 
	double omega_;          //周期関数の角加速度
	double phi_;            //法線方向と接線方向の周期関数の位相差
    double phi_fr_;         //前脚と後ろ足の位相差
    double center_z_;      //ボディーの高さ

    //ステアリングオフセット
    double steering_ofset_lf;   //左前   
    double steering_ofset_lr;   //左後ろ
    double steering_ofset_rr;   //右後ろ
    double steering_ofset_rf;   //右前

    //歩行モードへの変更時の脚の角度
    const float joint_2_limit = 0.2;
    const float joint_3_limit = 0.1;
    const float joint_position_margine = 0.1;  //目標値のマージン

    ros::NodeHandle nh_;

    std::array<ros::Publisher, 4> rollerwalker_joints_pub_lf_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_lr_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_rr_;
    std::array<ros::Publisher, 4> rollerwalker_joints_pub_rf_;

    

    //subscriber
    bool is_start_flag_;        //trueの時シミュレーションが動く
    bool is_rollerwalk_;     //true:ローラウォーク  false:歩行
    bool is_changing_mode_;  //モード変化中ならtrue
    bool is_complete_mode_change_; //モード変更が完了しているか
    ros::Subscriber is_start_flag_sub_,is_rollerwalk_flag_sub_; //flagのサブスクライバ
    ros::Subscriber current_center_z_sub_; //重心位置のサブスクライバ
    


};  