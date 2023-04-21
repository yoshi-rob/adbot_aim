/* This is a sample program for Cubic Control Library. */
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

double target = 2.25; // 正面

void messageCb( const std_msgs::Float64& aim){
  target = degToRad(aim.data) + 2.25;
// target = cmd_angle.data + 2.25;
}

ros::NodeHandle nh;
ros::NodeHandle pnh("~");
std_msgs::Float64 angle_diff;
std_msgs::Float64 angle;
std_msgs::Int16 duty;
// bool_msg.data = false;
ros::Publisher pub_angle_diff("angle_diff", &angle_diff);
ros::Publisher pub_angle("angle", &angle);
ros::Publisher pub_duty("duty", &duty);
ros::Subscriber<std_msgs::Float64> sub("aim", &messageCb );
// ros::Subscriber<std_msgs::Float64> sub("cmd_angle", &messageCb );

double capableDuty, Kp, Ki, Kd;
pnh.getParam("capableDuty", capableDuty);
pnh.getParam("Kp", Kp);
pnh.getParam("Ki", Ki);
pnh.getParam("Kd", Kd);

void setup()
{
  nh.initNode();
  nh.advertise(pub_angle_diff);
  nh.advertise(pub_angle);
  nh.advertise(pub_duty);
  nh.subscribe(sub);
  Cubic::begin();
  // Serial.begin(115200);
}

void loop()
{
  using namespace Cubic_controller;
  static Velocity_PID velocityPID(3, 0, encoderType::inc, 2048 * 4, 0.5, 0.5, 0.5, 0.1, 0.4, false, true);
  static Position_PID positionPID(3, 0, encoderType::abs, AMT22_CPR, capableDuty, Kp, Ki, Kd, target, true, true);
  static bool stopFlag = false;
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
    positionPID.setTarget(target);
    positionPID.compute();
    duty.data = DC_motor::get(3);
  }
  Cubic::update();

  angle_diff.data = abs(positionPID.getTarget() - positionPID.getCurrent());
  angle.data = positionPID.getCurrent() - 2.25;
  angle.data = radToDeg(angle.data);
  pub_angle_diff.publish(&angle_diff);
  pub_angle.publish(&angle);
  pub_duty.publish(&duty);
  
  nh.spinOnce();
  delay(1);
}
