/* This is a sample program for Cubic Control Library. */
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

double target = 0; // 正面
double capableDuty = 0.1, Kp, Ki, Kd;

ros::NodeHandle nh;

void messageCb( const std_msgs::Float64& aim){
  target = degToRad(aim.data);
// target = cmd_angle.data;
}

void messageCb_Kp( const std_msgs::Float64& sub_Kp){
  Kp = sub_Kp.data;
}

void messageCb_Ki( const std_msgs::Float64& sub_Ki){
  Kp = sub_Ki.data;
}

void messageCb_Kd( const std_msgs::Float64& sub_Kd){
  Kp = sub_Kd.data;
}

std_msgs::Float64 angle_diff;
std_msgs::Float64 angle;
std_msgs::Int16 duty;
ros::Publisher pub_angle_diff("angle_diff", &angle_diff);
ros::Publisher pub_angle("angle", &angle);
ros::Publisher pub_duty("duty", &duty);
ros::Subscriber<std_msgs::Float64> sub("aim", &messageCb );
ros::Subscriber<std_msgs::Float64> sub_Kp("Kp", &messageCb_Kp );
ros::Subscriber<std_msgs::Float64> sub_Ki("Ki", &messageCb_Ki );
ros::Subscriber<std_msgs::Float64> sub_Kd("Kd", &messageCb_Kd );

void setup()
{
  nh.getHardware()->setBaud(2000000);
  nh.initNode();
  nh.advertise(pub_angle_diff);
  nh.advertise(pub_angle);
  nh.advertise(pub_duty);
  nh.subscribe(sub);
  nh.subscribe(sub_Kp);
  nh.subscribe(sub_Ki);
  nh.subscribe(sub_Kd);
  Cubic::begin();
}

void loop()
{
  using namespace Cubic_controller;
  nh.spinOnce();
  
  static Velocity_PID velocityPID(3, 0, encoderType::inc, 2048 * 4, 0.5, 0.5, 0.5, 0.1, 0.4, false, false);
  static Position_PID positionPID(3, 0, encoderType::abs, AMT22_CPR, capableDuty, Kp, Ki, Kd, degToRad(target), true, false);
  static bool stopFlag = false;
  positionPID.setGains(Kp, Ki, Kd);
  /*
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    double value = Serial.readStringUntil('\n').toDouble();
    if (c == 'p')
    {
      velocityPID.setKp(value);
      positionPID.setKp(value);
    }
    else if (c == 'i')
    {
      velocityPID.setKi(value);
      positionPID.setKi(value);
    }
    else if (c == 'd')
    {
      velocityPID.setKd(value);
      positionPID.setKd(value);
    }
    else if (c == 's')
    {
      velocityPID.setTarget(value);
      positionPID.setTarget(degToRad(value));
    }
    else
    {
      stopFlag = true;
    }
  }
  */
  if (stopFlag)
  {
    // Serial.println("stopping...");
    for (int i = 0; i < 8; i++)
    {
      DC_motor::put(i, 0);
    }
  }
  else
  {
    // velocityPID.compute();
    positionPID.setTarget(degToRad(target));
    positionPID.compute();
    duty.data = DC_motor::get(3);
  }
  Cubic::update();

  angle_diff.data = abs(positionPID.getTarget() - positionPID.getCurrent());
  angle.data = positionPID.getCurrent();
  angle.data = radToDeg(angle.data);
  pub_angle_diff.publish(&angle_diff);
  pub_angle.publish(&angle);
  pub_duty.publish(&duty);
}
