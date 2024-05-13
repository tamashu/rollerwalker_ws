#include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>

// Map for movement keys
// std::map<char, std::vector<float>> moveBindings
// {
//   {'i', {1, 0, 0, 0}},
//   {'o', {1, 0, 0, -1}},
//   {'j', {0, 0, 0, 1}},
//   {'l', {0, 0, 0, -1}},
//   {'u', {1, 0, 0, 1}},
//   {',', {-1, 0, 0, 0}},
//   {'.', {-1, 0, 0, 1}},
//   {'m', {-1, 0, 0, -1}},
//   {'O', {1, -1, 0, 0}},
//   {'I', {1, 0, 0, 0}},
//   {'J', {0, 1, 0, 0}},
//   {'L', {0, -1, 0, 0}},
//   {'U', {1, 1, 0, 0}},
//   {'<', {-1, 0, 0, 0}},
//   {'>', {-1, -1, 0, 0}},
//   {'M', {-1, 1, 0, 0}},
//   {'t', {0, 0, 1, 0}},
//   {'b', {0, 0, -1, 0}},
//   {'k', {0, 0, 0, 0}},
//   {'K', {0, 0, 0, 0}}
// };


// Reminder message
const char* msg = "prease press [s] key to start(stop) simulation";
char key(' ');

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  bool pre_start_flag =false;
  bool pre_rollerwalk_flag =true;
  // Init ROS node
  ros::init(argc, argv, "teleop_twist_keyboard");
  ros::NodeHandle nh;

  // Init cmd_vel publisher
  ros::Publisher is_start_pub = nh.advertise<std_msgs::Bool>("is_start_flag", 1);
  ros::Publisher is_rollerwalk_pub = nh.advertise<std_msgs::Bool>("is_rollerwalk_flag", 1);
  std_msgs::Bool start_flag_msg , rollerwalk_flag_msg;



  printf("%s", msg);
  while(true){

    // Get the pressed key
    key = getch();

    // If the key corresponds to a key in moveBindings
    ROS_INFO("input_key: %c", key);

    if(key =='s'){  //起動スタート
      if(pre_start_flag==false){
        start_flag_msg.data = true;
        is_start_pub.publish(start_flag_msg);
        pre_start_flag = true;
      }
      else{
        start_flag_msg.data = false;
        is_start_pub.publish(start_flag_msg);
        pre_start_flag = false;
      }
    }
    else if(key =='c'){ //モードチェンジ
      if(pre_rollerwalk_flag==false){
        rollerwalk_flag_msg.data = true;
        is_rollerwalk_pub.publish(rollerwalk_flag_msg);
        pre_rollerwalk_flag = true;
      }
      else{
        rollerwalk_flag_msg.data = false;
        is_rollerwalk_pub.publish(rollerwalk_flag_msg);
        pre_rollerwalk_flag = false;
      }
    }

    // Otherwise, set the robot to stop
    else
    {
      // If ctrl-C (^C) was pressed, terminate the program
      if (key == '\x03')
      {
        printf("\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n");
        break;
      }
    }
    ros::spinOnce();
  }

  return 0;
}
