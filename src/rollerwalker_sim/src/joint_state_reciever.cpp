#include "rollerwalker_sim/joint_state_reciver.hpp"

JointStateReciever::JointStateReciever(){
    joint_states_sub_ = nh_.subscribe("/rollerwalker/joint_states", 0,&JointStateReciever::jointSubCalback_, this);
    joint_positions_pubs_[0] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_lf", 5);
    joint_positions_pubs_[1] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_lr", 5);
    joint_positions_pubs_[2] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_rr", 5);
    joint_positions_pubs_[3] = nh_.advertise<std_msgs::Float32MultiArray>("joint_positions_rf", 5);
    wheel_velocity_pub_      = nh_.advertise<std_msgs::Float32MultiArray>("wheel_velocity", 5);
    timer_ =nh_.createTimer(ros::Duration(0.1), &JointStateReciever::timerJointPositionPublishCalback, this);
}

void JointStateReciever::jointSubCalback_(const sensor_msgs::JointState::ConstPtr& msg){
    std::vector<std::string> joint_names = msg->name;
    std::vector<double> joint_position = msg->position;
    std::vector<double> joint_velocity = msg->velocity;
    
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
        else{//タイヤのポジション
            std::string joint_name =joint_names[i];
            std::string position =  joint_name.substr(joint_name.size()-2);   //jointの位置の取得
            if(position =="lf"){
                wheel_velocity_[0] = joint_velocity[i];
            }
            else if(position =="lr"){
                wheel_velocity_[1] = joint_velocity[i];
            }
            else if(position =="rr"){
                wheel_velocity_[2] = joint_velocity[i];
            }
            else if(position =="rf"){
                wheel_velocity_[3] = joint_velocity[i];
            }
        }
    }
}

void JointStateReciever::timerJointPositionPublishCalback(const ros::TimerEvent& e){
    std_msgs::Float32MultiArray position_msg;
    std_msgs::Float32MultiArray velocity_msg;
    //jointのポジションのpub
    for(int i=0;i<joint_positions_pubs_.size();i++){
        std::vector<float> pub_data(joint_positions_[i].begin(),joint_positions_[i].end());
        position_msg.data =pub_data;
        joint_positions_pubs_[i].publish(position_msg);
    }
    //wheel_velocityのpub
    std::vector<float> pub_data(wheel_velocity_.begin(),wheel_velocity_.end());
    velocity_msg.data =pub_data;
    wheel_velocity_pub_.publish(velocity_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Joint");
  JointStateReciever joint_state_reciver;
  ros::spin();
  return 0;
}