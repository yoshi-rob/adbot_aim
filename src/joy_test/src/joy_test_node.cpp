#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

//cmd_shooting_duty int32
//cmd_angle float64
//cmd_

std_msgs::Float64MultiArray aim_list;
std_msgs::Float64 aim;

ros::Publisher pub_aim;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  // 処理内容を記述
  ROS_INFO("axes[0]:%f", joy_msg.axes[0]);   // スティック0の状態を表示 (0 ～ 1)
  ROS_INFO("Button[0]:%d", joy_msg.buttons[0]);  // ボタン0の状態を表示 (0 or 1)

  if(joy_msg.buttons[0]==1) aim.data = aim_list.data[0];
  if(joy_msg.buttons[1]==1) aim.data = aim_list.data[1];
  if(joy_msg.buttons[2]==1) aim.data = aim_list.data[2];
  if(joy_msg.buttons[3]==1) aim.data = aim_list.data[3];

  pub_aim.publish(aim);
  std::cout << aim.data << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_test_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber sub_joy = nh.subscribe("joy", 10, joy_callback);
  pub_aim = nh.advertise<std_msgs::Float64>("aim",10);
  int angle1;
  int angle2;
  int angle3;
  int angle4;
  pnh.getParam("angle1", angle1);
  pnh.getParam("angle2", angle2);
  pnh.getParam("angle3", angle3);
  pnh.getParam("angle4", angle4);

  aim_list.data.resize(4);
  aim_list.data[0] = angle1;
  aim_list.data[1] = angle2;
  aim_list.data[2] = angle3;
  aim_list.data[3] = angle4;

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
