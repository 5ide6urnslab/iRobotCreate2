/*************************************************************************
 * File Name          : iRobotCreate2
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.02
 * Date               : 9/17/2015
 * Parts required     : Arduino UNO/MEGA 2560 R3 , iRobot Create 2
 * Description        : 
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2015 5ide6urns lab All right reserved.
 * History            : 8/12/2015 v1.00 Show Kawabata Create On.
 *                      9/16/2015 v1.01 Shingo Masui  [New func] connect the ROS
 *                      9/17/2015 v1.02 Show Kawabata [New func] add the drive func.
 **************************************************************************/
#include <iRobotCreate2.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

iRobotCreate2 roomba = iRobotCreate2(true, 2, 3);

std_msgs::String msg;
bool driveStatus = true;
uint32_t driveVelocity = 330;
void driveStatusCb( const std_msgs::Bool &status );
void driveVelocityCb( const std_msgs::UInt32 &velocity );

ros::NodeHandle nh;
ros::Publisher pub_msg("iRobot_msg", &msg);
ros::Subscriber<std_msgs::Bool> sub_driveStatus("iRobot_driveStatus", &driveStatusCb );
ros::Subscriber<std_msgs::UInt32> sub_driveVelocity("iRobot_driveVelocity", &driveVelocityCb );

void driveStatusCb( const std_msgs::Bool &status) {

  if (status.data) {
    msg.data = "driveStatus:true";
  } else {
    msg.data = "driveStatus:false";
  }
  pub_msg.publish(&msg);

  driveStatus = (bool)status.data;
}

void driveVelocityCb( const std_msgs::UInt32 &velocity) {
  char tmp[64] = {0};
  sprintf( tmp, "%s%d", "driverVelocity:", velocity.data );
  msg.data = tmp;
  pub_msg.publish(&msg);

  driveVelocity = (uint32_t)velocity.data;
}

void setup() {
  
  /*  [Note]: about the Serial communication.
   *    the rosserial uses the Hardware serial "Serial" whitch
   *    is D0(RX) and D1(TX). When the Arduino is used the multi serail
   *    communication, the Arduino Mega "Serail1" has to be used.
   *
   *      Serial  = rosserial    (Referance: ArduinoHardware.h)
   *      Serial1 = other device (ex. iRobot, XBee ZB)
   *
   */
  roomba.start();
  roomba.full();
  delay(1000);
  
  nh.initNode();
  nh.advertise(pub_msg);
  nh.subscribe(sub_driveStatus);
  nh.subscribe(sub_driveVelocity);
}

void loop() {

  nh.spinOnce();
  delay(50);
  
  int velocity = 0;
  if(driveStatus) velocity = (int)driveVelocity;
  roomba.drive(-velocity, velocity); 
}

