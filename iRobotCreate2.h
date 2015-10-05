#ifndef IROBOTCREATE2_H
#define IROBOTCREATE2_H

/*************************************************************************
 * File Name          : iRobotCreate2.h
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.00
 * Date               : 7/28/2015
 * Parts required     : Arduino UNO/MEGA 2560 R3 , iRobot Create 2
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
#include "SoftwareSerial.h"

#define clamp(value, min, max) (value < min ? min : value > max ? max : value)


class iRobotCreate2 {

public:

    /**********************************************************************
     * Constructor
     **********************************************************************/
	iRobotCreate2();
    iRobotCreate2(bool useSoftSerial, byte rx, byte tx);


    /**********************************************************************
     * Getting Started Commands
     **********************************************************************/
	void start();
	void stop();

    /**********************************************************************
     * Mode Commands
     **********************************************************************/
	void safe();
	void full();
	void passive();

    /**********************************************************************
     * Cleaning Commands
     **********************************************************************/
    // coming soon...
    
	
    /**********************************************************************
     * Actuator Commands
     **********************************************************************/
	void drive(int right, int left);
    

    /**********************************************************************
     * Input Commands
     **********************************************************************/
    // coming soon...

protected:
    bool usingSoftSerial;
    SoftwareSerial softSerial;


private:
    // none
};

#endif