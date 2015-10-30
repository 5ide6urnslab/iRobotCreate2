#ifndef ROBOTARM_H
#define ROBOTARM_H

/*************************************************************************
 * File Name          : RobotArm.h
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v0.01
 * Date               : 10/26/2015
 * Parts required     : Arduino UNO R3 , Sparkfun TB6612FNG Breakout,
 *                      Makeblock Robotic Arm Add-on Pack, 
 *                      DC/DC Converter MCW03-05S12, Potention Meter(10k)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2015 5ide6urns lab All right reserved.
 **************************************************************************/

// Arduino version.
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


class RobotArm {

public:

    /**********************************************************************
     * Constructor
     **********************************************************************/
    RobotArm(int stby, int pwmA, int aIn1, int aIn2, int potention);


    /**********************************************************************
     * Robot Arm Control
     **********************************************************************/
    void move(int speed); // value between -255 and 255.
    void stop();

    /**********************************************************************
     * Robot Arm Control Status
     **********************************************************************/
    void getMotorPosition(int* value);
    
protected:
    // none

private:
    int _stby;
    int _pwmA;
    int _aIn1;
    int _aIn2;
    int _potention;
};

#endif