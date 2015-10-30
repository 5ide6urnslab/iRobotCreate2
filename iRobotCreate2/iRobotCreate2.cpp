
/*************************************************************************
 * File Name          : iRobotCreate2.cpp
 * Author             : Show Kawabata
 * Version            : v1.00
 * Date               : 9/15/2015
 * Parts required     : Arduino UNO/MEGA 2560 R3 , iRobot Create 2
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2015 5ide6urns lab All right reserved.
 **************************************************************************/

#include "iRobotCreate2.h"

#define ARDUINO_UNO  // For the Arduino type


/*! *******************************************************************
 *  @fn         iRobotCreate2
 *  @brief      It is Constructor for iRobotCreate2 class to 
 *              initialize the data and initializing process.
 *              This constructor is for the Hardware Serial.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
iRobotCreate2::iRobotCreate2(){
    
    usingSoftSerial = false;

#if defined(ARDUINO_UNO)
	Serial.begin(19200);
#else
	Serial1.begin(19200);
#endif
    
}

/*! *******************************************************************
 *  @fn         iRobotCreate2
 *  @brief      It is Constructor for iRobotCreate2 class to
 *              initialize the data and initializing process.
 *              This constructor is for the Software Serial.
 *
 *  @param[in]  useSoftSerial:     the "true" is for using software serial
 *                                 the "false" is for not use software serial
 *              rxPin:             Serial receive pin number for software serial
 *              txPin:             Serial transmit pin number for software serial
 *              baudRateChangePin: baudrate for software serial
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
iRobotCreate2::iRobotCreate2(bool useSoftSerial, byte rx, byte tx){
    
    usingSoftSerial = useSoftSerial;
    
	if (useSoftSerial){
		softSerial = SoftwareSerial(rx, tx);
		softSerial.begin(19200);
	}
	else {

#if defined(ARDUINO_UNO)
		Serial.begin(19200);
#else
        Serial1.begin(19200);
#endif

	}
}


/**********************************************************************
 * [Note]: Getting Started Commands
 *  The following I/F start the Open Interface and get it ready for use.
 **********************************************************************/

/*! *******************************************************************
 *  @fn         start
 *  @brief      It starts the Open Interface(OI).
 *              You must always send the Start command before sending
 *              any other commands to the OI.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
void iRobotCreate2::start(){
    
    if(usingSoftSerial){
        softSerial.write(128);
    }
    else{

#if defined(ARDUINO_UNO)
        Serial.write(128);
#else
        Serial1.write(128);
#endif
        
    }
}


/*! *******************************************************************
 *  @fn         stop
 *  @brief      It stops the Open Interface(OI). All stremas will stop
 *              and the robot will no longer respond to commands.
 *              Use this command when you are finished working with
 *              the robot.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
void iRobotCreate2::stop(){
    
    if(usingSoftSerial){
        softSerial.write(173);
    }
    else{
#if defined(ARDUINO_UNO)
        Serial.write(173);
#else
        Serial1.write(173);
#endif
        
    }
}



/**********************************************************************
 * [Note]: Mode Commands
 *  The iRobot Create 2 has four operating modes: Off, Passive, Safe, 
 *  and Full. iRobot Create 2 powers on in the Off mode. The following
 *  commands change iRobot Create 2’s Opne Interface(OI) mode.
 **********************************************************************/

/*! *******************************************************************
 *  @fn         safe
 *  @brief      It puts the Open Interface(OI) into Safe mode, 
 *              enabling user control of iRobot Create 2. It turns off
 *              all LEDs. The OI can be in Passive, Safe, or Full mode
 *              to accept this command. If a safety condition occurs
 *              iRobot Create 2 reverts automatically to Passive mode.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
void iRobotCreate2::safe(){
    
    if(usingSoftSerial){
        softSerial.write(131);
    }
    else{

#if defined(ARDUINO_UNO)
        Serial.write(131);
#else
        Serial1.write(131);
#endif
        
    }
}

/*! *******************************************************************
 *  @fn         full
 *  @brief      It gives you complete control over iRobot Create 2 by
 *              putting the Open Interface(OI) into Full mode, and
 *              turning off the cliff, wheel-drop and internal charger
 *              safety features. That is, in Full mode, iRobot Create 2
 *              executes any command that you send it, even if the
 *              internal charger is plugged in, or command triggers a
 *              cliff or wheel drop condition.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
void iRobotCreate2::full(){

    if(usingSoftSerial){
        softSerial.write(132);
    }
    else{

#if defined(ARDUINO_UNO)
        Serial.write(132);
#else
        Serial1.write(132);
#endif
        
    }
}

/*! *******************************************************************
 *  @fn         passive
 *  @brief
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       7/28/2015  v1.00:  Create on.
 ***********************************************************************/
void iRobotCreate2::passive(){
	start();
}


/**********************************************************************
 * [Note]: Cleaning Commands
 *  The following are commands to start iRobot Create 2’s built-in
 *  cleaning modes and set the clock and schedule.
 **********************************************************************/
// coming soon...



/**********************************************************************
 * [Note]: Actuator Commands
 *  The following commands control Roomba’s actuators: wheels, brushes,
 *  vacuum, speaker, LEDS, and buttons.
 **********************************************************************/

/*! *******************************************************************
 *  @fn         drive
 *  @brief      It lets you control the forward and backward motion of
 *              iRobot Create 2’s drive wheels independently. It takes
 *              four data bytes, which are interpreted as two 16-bit
 *              signed values using two’s complement. The first two bytes
 *              specify the velocity of the right wheel in millimeters
 *              per second (mm/s), with the high byte sent first.
 *              The next two bytes specify the velocity of the left wheel,
 *              in the same format. A positive velocity makes that wheel
 *              drive forward, while a negative velocity makes it drive
 *              backward.
 *
 *              Serial sequence:
 *              [145][Right velocity high byte][Right velocity low byte]
 *              [Left velocity high byte][Left velocity low byte]
 *
 *  @param[in]  right:  the right wheel velocity(-500 - 500mm/s)
 *  @param[in]  left :  the left wheel velocity(-500 - 500mm/s)
 *  @return     void
 *  @version    v1.02
 *  @date       7/28/2015  v1.00:  Create on.
 *              9/15/2015  v1.02:  [New func] add Software Serial function.
 ***********************************************************************/
void iRobotCreate2::drive(int right, int left){
	clamp(right, -500, 500);    // Right wheel velocity(-500 - 500mm/s)
	clamp(left, -500, 500);     // Left wheel velocity(-500 - 500mm/s)

    if(usingSoftSerial){
        softSerial.write(145);
		softSerial.write(right >> 8);
		softSerial.write(right);
		softSerial.write(left >> 8);
		softSerial.write(left);
    }
    else{
#if defined(ARDUINO_UNO)
        Serial.write(145);
        Serial.write(right >> 8);
        Serial.write(right);
        Serial.write(left >> 8);
        Serial.write(left);
#else
        Serial1.write(145);
        Serial1.write(right >> 8);
        Serial1.write(right);
        Serial1.write(left >> 8);
        Serial1.write(left);
#endif
        
    }
}



/**********************************************************************
 * [Note]: Input Commands
 *  The following commands control Roomba’s actuators: wheels, brushes,
 *  vacuum, speaker, LEDS, and buttons.
 **********************************************************************/
// coming soon...
