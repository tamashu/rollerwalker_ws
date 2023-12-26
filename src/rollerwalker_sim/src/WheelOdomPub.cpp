#include "rollerwalker_sim/WheelOdomPub.hpp"
#include <cmath>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

WheelOdomPub::WheelOdomPub():nh_("~")
{
    nh_.getParam("/rollerwalker_param/l2", l2_);
    nh_.getParam("/rollerwalker_param/l3", l3_);
    nh_.getParam("/rollerwalker_param/l4", l4_);
    nh_.getParam("/rollerwalker_param/wheel_radius", wheel_radius_);
    nh_.getParam("/rollerwalker_param/body_width", body_width_);
    nh_.getParam("/publish_frequency", publish_frequency_);

    //jointの状態を受け取るサブスクライバの設定
    joint_positions_lf_sub_ = nh_.subscribe("/joint_positions_lf", 5, &WheelOdomPub::jointPositionLFCallback_, this);
    joint_positions_lr_sub_ = nh_.subscribe("/joint_positions_lr", 5, &WheelOdomPub::jointPositionLRCallback_, this);
    joint_positions_rr_sub_ = nh_.subscribe("/joint_positions_rr", 5, &WheelOdomPub::jointPositionRRCallback_, this);
    joint_positions_rf_sub_ = nh_.subscribe("/joint_positions_rf", 5, &WheelOdomPub::jointPositionRFCallback_, this);
    //速度関係のサブスクライバの設定
    wheel_velocity_sub_     = nh_.subscribe("/wheel_velocity"    , 5, &WheelOdomPub::wheelVelocityCallback_, this);
    true_position_sub_      = nh_.subscribe("/true_position"    , 1, &WheelOdomPub::truePositionCallback_, this);

    //publidherの設定
    velocity_pub_ = nh_.advertise<std_msgs::Float64>("/velocity", 5);
    true_velocity_pub_ = nh_.advertise<std_msgs::Float64>("/true_velocity", 5);
    true_yaw_vel_pub_ = nh_.advertise<std_msgs::Float64>("/true_yaw_velocity", 5);
    center_z_pub_ = nh_.advertise<std_msgs::Float64>("/current_center_z", 1);

    //timercallbavkの設定
    timer_ =nh_.createTimer(ros::Duration(1/publish_frequency_), &WheelOdomPub::timerWheelVelocityPubCallback_, this);
    //平均値フィルターの初期化
    for(int i = 0;i<4;i++){
        filtering_wheel_velocity_[i].fill(0);
    }
    wheel_counters_.fill(0);
    wheel_sums_.fill(0.0);

    //中央値フィルターの初期化
    for(int i =0;i<3;i++){
        filtering_velocity_[i].clear();
        filtering_rotation_velocity_[i].clear();
    }
    //初期時刻
    pre_time = ros::Time::now();
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
        wheel_velocity_[i] = calMovingAverage(msg.data[i],filtering_wheel_velocity_[i],wheel_counters_[i],wheel_sums_[i]);    
    }
}
void WheelOdomPub::truePositionCallback_(const  nav_msgs::Odometry& msg){
    std::array<float, 3> true_position;        //x,y,z
    std::array<float, 3> true_rotation;        //roll picth yaw
    std::array<float, 3> delta_position;       //delta_x,delta_y,delta_z,
    std::array<float, 3> delta_rotation;       //delta_roll, delta_pich, delta_yaw

    std_msgs::Float64 true_velocity_msg;
    std_msgs::Float64 true_rotation_velocity_msg;

    true_position[0] = msg.pose.pose.position.x;
    true_position[1] = msg.pose.pose.position.y;
    true_position[2] = msg.pose.pose.position.z;

    true_rotation = quartanionToRPY(msg);
    
    float delta_t = (ros::Time::now() - pre_time).toSec();
    if(delta_t>0){ //delta_t=0を避けるため
        for(int i=0;i<3;i++){
            //速度計算
            delta_position[i] = true_position[i] - pre_true_position_[i];
            float v_tmp = delta_position[i] / delta_t;       //v = dx /dt
            V_[i] = medianFilter(filtering_velocity_[i],v_tmp);
            pre_true_position_[i] = true_position[i]; //現情報を前情報に代入

            ////回転の計算
            delta_rotation[i] = true_rotation[i] - pre_true_rotation_[i];
            float rotation_v_tmp = delta_rotation[i] / delta_t;       //v = dx /dt
            rotation_v_[i] = medianFilter(filtering_rotation_velocity_[i],rotation_v_tmp);
            pre_true_rotation_[i] = true_rotation[i]; //現情報を前情報に代入
        }

        //各軸に対する速度
        // std::vector<float> pub_data(V_.begin(),V_.end());
        // true_velocity_msg.data =pub_data;

        //速度
        float V = sqrt(pow(V_[0],2)+pow(V_[1],2));

        //速度のパブリッシュ
        true_velocity_msg.data = V;
        true_velocity_pub_.publish(true_velocity_msg);

        //yaw角の速度のパブリッシュ
        true_rotation_velocity_msg.data = rotation_v_[2];
        true_yaw_vel_pub_.publish(true_rotation_velocity_msg);
        
        // ROS_INFO("yaw:%f  delta_yaw:%f ",true_rotation[2], rotation_v_[2]);
    }
    else{   //0割りしてしまうときは位置だけ代入する
        for(int i=0;i<3;i++){
            pre_true_position_[i] = true_position[i]; //現情報を前情報に代入
        }
    }
    // ROS_INFO("z:%f",true_position[2]);
    pre_time = ros::Time::now();
}


//odometryのパブリッシュ
void WheelOdomPub::timerWheelVelocityPubCallback_(const ros::TimerEvent& e){
    std_msgs::Float64 velocity_msg;
    std_msgs::Float64 current_center_z_msg;

    double velocity_left = wheel_radius_*calAverage(wheel_velocity_[0]*cos(joint_position_lf_[0]),wheel_velocity_[1]*cos(joint_position_lr_[0]));       //左のタイヤの速度（平均）
    double velocity_right = -wheel_radius_ * calAverage(wheel_velocity_[2]*cos(joint_position_rr_[0]),wheel_velocity_[3]*cos(joint_position_rf_[0]));     //右のタイヤの速度（平均）（符号はマイナス）
    
    double front_current_z = calAverage(calZ_(joint_position_lf_[1],joint_position_lf_[2],joint_position_lf_[3]),calZ_(joint_position_rf_[1],joint_position_rf_[2],joint_position_rf_[3])); //前の高さの平均

    velocity_msg.data =calAverage(velocity_left , velocity_right);
    velocity_pub_.publish(velocity_msg);

    // 現在の重心位置のパブリッシュ
    current_center_z_msg.data = front_current_z;
    center_z_pub_.publish(current_center_z_msg);
};

double WheelOdomPub::calD_(double theta_2,double theta_3,double theta_4){
    double x_e = l2_ * cos(theta_2) + l3_ * sin(theta_2 + theta_3) + l4_ * sin(theta_2 + theta_3 + theta_4);
    return x_e;
}
double WheelOdomPub::calZ_(double theta_2,double theta_3,double theta_4){
    double z_e = -(l2_ * sin(theta_2) - l3_ * cos(theta_2 + theta_3) - l4_ * cos(theta_2 + theta_3 + theta_4))+wheel_radius_;
    return z_e;
}

std::array<float,3> WheelOdomPub::quartanionToRPY(nav_msgs::Odometry msg){
    geometry_msgs::Pose2D pose2d;
    pose2d.x = msg.pose.pose.position.x;
    pose2d.y = msg.pose.pose.position.y;

    tf::Quaternion q(msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    std::array<float, 3>ret = {(float)roll, (float)pitch, (float)yaw};
    return ret;
}

double WheelOdomPub::calAverage(double input1, double input2){
    double ret = (input1+input2)/2;
    return ret;
}
float WheelOdomPub::calMovingAverage(float input_value, std::array<float,NUM_OF_FILTERING>&filterring_array, int& count, float&  sum){
    if (count == NUM_OF_FILTERING){
        count = 0;
    } 
    sum -= filterring_array[count];
    filterring_array[count] = input_value;
    sum += filterring_array[count];
    count++;
    float average = sum / (float)NUM_OF_FILTERING;
    //ROS_INFO("input:%f  sum[i]: %.5f  wheel_counters[i]:%d return:%f ",input_value,sum,count,average);
    return average;
}

float WheelOdomPub::medianFilter(std::vector<float>&filtering_value,float input){
    filtering_value.push_back(input);
    int num_of_filtering_value = filtering_value.size();
    std::vector<float>copied_vector; //ソートすためのコピー

    if(num_of_filtering_value==MAX__MEDIAN_FILTERING_NUM){ //既にFILTERINGが10個になっていたら
        filtering_value.erase(filtering_value.begin()); //最初の要素を削除
    }
    std::copy(filtering_value.begin(),filtering_value.end(),std::back_inserter(copied_vector));
    int copied_vector_size = copied_vector.size();

    std::sort(std::begin(copied_vector),std::end(copied_vector));
    
    if(copied_vector_size %2 == 0){//要素が偶数なら真ん中の2つの要素の平均を返す
        return (copied_vector[copied_vector_size/2] +copied_vector[copied_vector_size/2-1])/2;
    }
    else{//奇数なら真ん中の要素で返す
        return copied_vector[copied_vector_size/2];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheel_odom_pub");
    WheelOdomPub wheel_odom_pub;
    ros::spin();
    return 0;
}