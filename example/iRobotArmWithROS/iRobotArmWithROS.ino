/*************************************************************************
 * File Name          : iRobotArmWithROS
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.01
 * Date               : 11/18/2015
 * Parts required     : Arduino UNO R3, Sparkfun TB6612FNG Breakout,
 *                      Makeblock Robotic Arm Add-on Pack,
 *                      DC/DC Converter MCW03-05S12
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2015 5ide6urns lab All right reserved.
 * History            : 10/26/2015 v1.00 Show Kawabata Create On.
 *                      11/18/2015 v1.01 Show Kawabata Add rosserial.
 **************************************************************************/
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>

#include <iRobotArm.h>


// stby, PWMA, AIN1, AIN2
iRobotArm rArm = iRobotArm(10, 3, 9, 8);

ros::NodeHandle nh;

std_msgs::String msg;
ros::Publisher pub_msg("iRobot-msg", &msg);

uint32_t _armPosition;
void armPositionCb( const std_msgs::UInt32 &armPosition );
ros::Subscriber<std_msgs::UInt32> sub_armPosition("Arm-drivePosition", &armPositionCb );


void armPositionCb( const std_msgs::UInt32 &armPosition) {
  char tmp[64] = {0};
  sprintf( tmp, "%s%d", "Arm-drivePosition:", armPosition.data );
  msg.data = tmp;
  pub_msg.publish(&msg);

  _armPosition = (uint32_t)armPosition.data;
}


void setup(){
 
//  Serial.begin(115200); 
  
  nh.initNode();
  nh.subscribe(sub_armPosition);
  nh.advertise(pub_msg);
 
}


void loop(){
  
  nh.spinOnce();
  delay(1);
  
  if(_armPosition == 0x00){
      rArm.move(-30);    // counter-clockwise(UP)
  }
  else if(_armPosition == 0x01){
      rArm.move(30);    // clockwise(Down)
  }
  else{
      rArm.stop();    
  }
}
