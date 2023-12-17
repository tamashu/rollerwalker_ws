#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <array>
#include "rollerwalker_sim/BaseRollerwalker.hpp"
#include <std_msgs/Float64.h>


class RollerwalkerDriver : public BaseRollerwalker
{
public:
    RollerwalkerDriver(double d_0, double theta_0, double omega, double phi,  double phi_fr, double center_z, bool is_rollerWalk);
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
    const double PI =3.1415926535897932384626433832795;
    //歩行モードへの変更時の脚の角度
    const float joint_2_limit = 0.2;
    const float joint_3_limit = 0.1;
    const float joint_position_margine = 0.1;  //目標値のマージン

    ros::NodeHandle nh_;
    double t_;       //脚軌道の時刻
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