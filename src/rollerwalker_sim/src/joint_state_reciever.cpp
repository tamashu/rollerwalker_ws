#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
class JointStateReciever
{
public:
    JointStateReciever();
    void jointSubCalback_(const sensor_msgs::JointState::ConstPtr& msg);
    void timerJointPositionPublishCalback(const ros::TimerEvent& e);
private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_states_sub_;
    ros::Publisher joint_positions_lf_pub_,joint_positions_lr_pub_,joint_positions_rr_pub_,joint_positions_rf_pub_;
    ros::Timer timer_;

    std::array<float, 4> joint_lf_positions_{}; // rad
    std::array<float, 4> joint_lr_positions_{}; // rad
    std::array<float, 4> joint_rr_positions_{}; // rad
    std::array<float, 4> joint_rf_positions_{}; // rad
    std::array<std::array<float, 4>,4> joint_positions_{joint_lf_positions_,joint_lr_positions_,joint_rr_positions_,joint_rf_positions_};

    std::array<ros::Publisher, 4> joint_positions_pubs_;

};

JointStateReciever::JointStateReciever(){
    joint_states_sub_ = nh_.subscribe("/rollerwalker/joint_states", 0,&JointStateReciever::jointSubCalback_, this);
    joint_positions_pubs_[0] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_lf", 5);
    joint_positions_pubs_[1] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_lr", 5);
    joint_positions_pubs_[2] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_rr", 5);
    joint_positions_pubs_[3] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_rf", 5);
    timer_ =nh_.createTimer(ros::Duration(0.1), &JointStateReciever::timerJointPositionPublishCalback, this);
}

void JointStateReciever::jointSubCalback_(const sensor_msgs::JointState::ConstPtr& msg){
    std::vector<std::string> joint_names = msg->name;
    std::vector<double> joint_position = msg->position;
    
    //jointのpositionの取得
    for(int i=0;i<joint_names.size();i++){
        if(joint_names[i].substr(0,5)=="joint"){
            int joint_num = int(joint_names[i][6] - '0');       //joint番号の取得
            std::string position =  joint_names[i].substr(8);   //jointの位置の取得
            if(position =="lf"){
                joint_positions_[0][joint_num-1] = joint_position[i];
            }
            else if(position =="lr"){
                joint_positions_[1][joint_num-1] = joint_position[i];
            }
            else if(position =="rr"){
                joint_positions_[2][joint_num-1] = joint_position[i];
            }
            else if(position =="rf"){
                joint_positions_[3][joint_num-1] = joint_position[i];
            }
        }
    }

}

void JointStateReciever::timerJointPositionPublishCalback(const ros::TimerEvent& e){
    std_msgs::Float32MultiArray position_msg;
    for(int i=0;i<joint_positions_pubs_.size();i++){
        std::vector<float> pub_data(joint_positions_[i].begin(),joint_positions_[i].end());
        position_msg.data =pub_data;
        joint_positions_pubs_[i].publish(position_msg);
    }
    // position_msg.data = joint_positions_[0];
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Joint");
  JointStateReciever joint_state_reciver;
  ros::spin();
  return 0;
}