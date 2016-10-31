
/*************************************************************************
 * File Name          : testApp.h
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.05
 * Date               : 10/17/2016
 * Parts required     : ofxGui, ofxOsc, ofxXmlSettings, ofxCsv
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 *
 * History            : 10/03/2016 v1.00 Show Kawabata Create On.
 *                      10/06/2016 v1.01 Show Kawabata [New func] Logging.
 *                      10/07/2016 v1.02 Show Kawabata [Bug fix] Released Vector.
 *                      10/17/2016 v1.04 Show Kawabata [Refact] Serial Transmit.
 *                      10/17/2016 v1.05 Show Kawabata [New func] Serial Receive.
 *************************************************************************/

#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxCsv.h"
#include "zumo_message_id.h"

using namespace ZumoMessageIds;
using namespace wng;

/**************************************************
 * Debug Definition.
 **************************************************/
#define DEBUG_ENABLED
#define DEBUG_PRINT(A) cout << (#A) << ":" << (A) << endl;

/**************************************************
 * CSV Definition.
 **************************************************/
#define DEF_CSV_TIMESTAMP      "TimeStamp"
#define DEF_CSV_CDS            "Cds Sensor"
#define DEF_CSV_VOLTAGE        "Voltage"
#define DEF_CSV_CURRENT        "Current"

/**************************************************
 * Serial Definition.
 **************************************************/
#define DEF_SERIAL_PORT         "/dev/cu.usbserial-DN00OK5F"
#define DEF_SERIAL_BAURATE      115200
#define DEF_SERIAL_MASK         0b11111111
#define DEF_SERIAL_PREFIX       0x24
#define DEF_SERIAL_SEARCH_TOTAL 3
#define DEF_SERIAL_SEARCH_NUM   2

/**************************************************
 * Gui Definition.
 **************************************************/
#define DEF_GUI_WINDOW_TITLE    "RobotSimuLink"
#define DEF_GUI_CONTROLLING     "CONTROLLING"
#define DEF_GUI_DEBUG           "DEBUG"
#define DEF_GUI_VISUAL          "VISUALIZATION"
#define DEF_GUI_DRIVE           "Drive Status"
#define DEF_GUI_SENSOR          "Sensor Status"
#define DEF_GUI_SEND            "Send Data"
#define DEF_GUI_RIGHT_VELOCITY  "Right Velocity"
#define DEF_GUI_LEFT_VELOCITY   "Left Velocity"
#define DEF_GUI_LOGGING         "Logging"
#define DEF_GUI_GEOMAGNETIC     "Geomagnetic Sensor"
#define DEF_GUI_GEO_X           "Geomagnetic X"
#define DEF_GUI_GEO_Y           "Geomagnetic Y"
#define DEF_GUI_GEO_Z           "Geomagnetic Z"
#define DEF_GUI_ENCODER         "Encoder"
#define DEF_GUI_LEFT_ENCODER    "Left Encoder"  
#define DEF_GUI_RIGHT_ENCODER   "Right Encoder"
#define DEF_GUI_ACCEL           "Acceleration"
#define DEF_GUI_ACCEL_X         "Acceleration X"
#define DEF_GUI_ACCEL_Y         "Acceleration Y"
#define DEF_GUI_ACCEL_Z         "Acceleration Z"
#define DEF_GUI_VELOCITY_MIN    (-500)
#define DEF_GUI_VELOCITY_MAX    500
#define DEF_GUI_VELOCITY_INIT   0
#define DEF_GUI_R_MIN           (-32767)
#define DEF_GUI_R_MAX           32767
#define DEF_GUI_G_MIN           (-200)
#define DEF_GUI_G_MAX           200


const short searchTable[DEF_SERIAL_SEARCH_TOTAL][DEF_SERIAL_SEARCH_NUM] = {
    {kZumoMsgIdMotorEncoder, 2},
    {kZumoMsgIdGeoMagnetism, 3},
    {kZumoMsgIdGeoMagnetism, 3}
};


class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
    /**************************************************
     * Gui.
     **************************************************/
    void onDrivePushed(bool &value);
    void onSensorPushed(bool &value);
    void onRightVelocityChanged(int &value);
    void onLeftVelocityChanged(int &value);
    void onSendDataPushed();
    void onLoggingPushed(bool &value);
    void onGeomagneticPushed(bool &value);
    void onEncoderPushed(bool &value);
    void onAccelPushed(bool &value);
    
    ofxPanel              _gui;
    ofxPanel              _gui1;
    ofxPanel              _gui2;
    
    ofxToggle             _drive;
    ofxToggle             _sensor;
    ofxToggle             _logging;
    ofxToggle             _encoder;
    ofxToggle             _geomagnetic;
    ofxToggle             _accel;
    
    ofxButton             _send;
    
    ofxIntSlider          _rVelocity;
    ofxIntSlider          _lVelocity;
    
    short                 _rData;
    short                 _lData;
    
    bool                  _isSending;
    bool                  _isSensor;
    bool                  _isLogging;
    bool                  _isGeomagnetic;
    bool                  _isEncoder;
    bool                  _isAccel;
    
    /**************************************************
     * Serial.
     **************************************************/
    void sendData(unsigned char id, short value[], size_t length);
    void recieveData(unsigned char id);
    
    ofSerial              _serial;

    /**************************************************
     * Csv.
     **************************************************/
    void createCsv();
    void updateCsv();
    
    ofxCsv                _csv;
    
    unsigned int          _row;
    unsigned int          _col;
    string                _fileName;

    /**************************************************
     * Sensing.
     **************************************************/
    vector<int>           _lEncoder;
    vector<int>           _rEncoder;
    vector<int>           _gX, _gY, _gZ;
    vector<int>           _aX, _aY, _aZ;
    
    unsigned short        _cds;
    unsigned short        _voltage;
    unsigned short        _current;
    
};
