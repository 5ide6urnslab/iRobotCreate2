
/*************************************************************************
 * File Name          : iRobotArm.cpp
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v0.01
 * Date               : 10/26/2015
 * Parts required     : Arduino UNO R3, Sparkfun TB6612FNG Breakout,
 *                      Makeblock Robotic Arm Add-on Pack,
 *                      DC/DC Converter MCW03-05S12, Potention Meter(10k)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2015 5ide6urns lab All right reserved.
 **************************************************************************/

#include "iRobotArm.h"


/*! *******************************************************************
 *  @fn         iRobotArm
 *  @brief      It is Constructor for RobotArm class to
 *              initialize the data and initializing process.
 *
 *  @param[in]  stby:       The STBY pin number for TB6612FNG Breakout.
 *              pwmA:       The PWMA pin number for TB6612FNG Breakout.
 *              aIn1:       The AIN1 pin number for TB6612FNG Breakout.
 *              aIn2:       The AIN2 pin number for TB6612FNG Breakout.
 *              potention:  The analog pin number for Potention Meter.
 *  @return     void
 *  @version    v1.00
 ***********************************************************************/
iRobotArm::iRobotArm(int stby, int pwmA, int aIn1, int aIn2, int potention){
    
    _stby      = stby;
    _pwmA      = pwmA;
    _aIn1      = aIn1;
    _aIn2      = aIn2;
    _potention = potention;
    
    pinMode(stby, OUTPUT);
    
    pinMode(pwmA, OUTPUT);
    pinMode(aIn1, OUTPUT);
    pinMode(aIn2, OUTPUT);
    
}


/*! *******************************************************************
 *  @fn         move
 *  @brief      It works the motor and changes the direction, speed.
 *
 *  @param[in]  speed:      The Motor speed value(PWM) between -255 and 255.
 *                              (0)          : stop
 *                              (1) - (255)  : clockwise
 *                              (-1) - (-255): couter-clockwise
 *
 *  @return     void
 *  @version    v1.00
 ***********************************************************************/
void iRobotArm::move(int speed){
    
    // disable STBY
    digitalWrite(_stby, HIGH);
    
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;
    
    boolean in1 = HIGH;
    boolean in2 = LOW;
    
    if(speed >= 0){

        // clockwise
        digitalWrite(_aIn1, in1);
        digitalWrite(_aIn2, in2);
        analogWrite(_pwmA, speed);
    }
    else{
        
        // couter-clockwise
        in1 = LOW;
        in2 = HIGH;

        digitalWrite(_aIn1, in1);
        digitalWrite(_aIn2, in2);
        analogWrite(_pwmA, -speed);
    }
}


/*! *******************************************************************
 *  @fn         stop
 *  @brief      It stops the motor.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 ***********************************************************************/
void iRobotArm::stop(){
    
    // enable STBY
    digitalWrite(_stby, LOW);
}

/*! *******************************************************************
 *  @fn         getMotorAngle
 *  @brief      It gets the Motor position from the Potention Meter.
 *
 *  @param[out] value:  Potention Meter value
 *  @return     void
 *  @version    v1.00
 ***********************************************************************/
void iRobotArm::getMotorPosition(int* value){
    
    *value = analogRead(_potention);
}
