#include <string>
#include <math.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#define NUM_OF_FILTERING 10

class WheelOdomPub{
public:
    WheelOdomPub();
    void jointPositionLFCallback_(const std_msgs::Float32MultiArray& msg);
    void jointPositionLRCallback_(const std_msgs::Float32MultiArray& msg);
    void jointPositionRRCallback_(const std_msgs::Float32MultiArray& msg);
    void jointPositionRFCallback_(const std_msgs::Float32MultiArray& msg);
    void wheelVelocityCallback_(const std_msgs::Float32MultiArray& msg);
    void timerWheelVelocityPubCallback_(const ros::TimerEvent& e);

private:
    double calD_(double theta_2,double theta_3,double theta_4);   //ジョイント2からリンク4の先端までの距離
    double calZ_(double theta_2,double theta_3,double theta_4);   //ボディの高さの計算
    double calAverage(double input1, double input2);              //平均の計算
    double calMovingAverage(double input_value, std::array<float,NUM_OF_FILTERING>filterring_array, int& count, float&  sum);              //平均の計算

    ros::NodeHandle nh_;
    ros::Timer timer_;

    double l2_;
    double l3_;
    double l4_;
    double wheel_radius_;
    double body_width_;

    //各関節の姿勢情報
    std::array<float,4>joint_position_lf_;
    std::array<float,4>joint_position_lr_;
    std::array<float,4>joint_position_rr_;
    std::array<float,4>joint_position_rf_;
    std::array<float,4>wheel_velocity_;

    //各脚の長さd
    std::array<float,4>d_;
    //移動平均関係
    std::array<std::array<float,NUM_OF_FILTERING>,4>filtering_wheel_velocity_;//フィルタリングの値を格納
    std::array<int,4>wheel_counters_;
    std::array<float,4>wheel_sums_;

    ros::Subscriber joint_positions_lf_sub_,joint_positions_lr_sub_,joint_positions_rr_sub_,joint_positions_rf_sub_,wheel_velocity_sub_; //各関節の状態のサブスクライバ
    ros::Publisher velocity_pub_;
};