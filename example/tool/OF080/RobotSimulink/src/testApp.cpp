
/*************************************************************************
 * File Name          : testApp.cpp
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.06
 * Date               : 11/08/2016
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
 *                      10/12/2016 v1.03 Show Kawabata [New func] Graph Display.
 *                      10/17/2016 v1.04 Show Kawabata [Refact] Serial Transmit.
 *                      10/17/2016 v1.05 Show Kawabata [New func] Serial Receive.
 *                      11/08/2016 v1.06 Show Kawabata [Refact] Common Message.
 *************************************************************************/

#include "testApp.h"


/*! *******************************************************************
 *  @fn         recieveData
 *  @brief      This function is the Packet Reciever for data coupling.
 *
 *  @param[in]  id          :   Packet ID.
 *  @return     void
 *  @version    v1.06
 *  @date       10/17/2016 v1.05 [New func] Serial Receive.
 *              11/08/2016 v1.06 [Refact] Common Message.
 **********************************************************************/
void testApp::recieveData(unsigned char id){

    vector<short> aData_;
    
    short         data_      = 0;
    short         checkSum_  = 0;
    unsigned char msb_       = 0;
    unsigned char lsb_       = 0;
    unsigned char searchNum_ = 0;


    for(int i = 0; i < DEF_SERIAL_SEARCH_TOTAL; ++i){
        if(id == searchTable[i][0]){
            searchNum_ = searchTable[i][1];
        }
    }
    
    for(int i = 0; i < searchNum_; ++i){
        msb_ = _serial.readByte();
        lsb_ = _serial.readByte();
        aData_.push_back((msb_ << 8) | lsb_);
    }
    
    for(int i = 0; i < aData_.size(); ++i){
        data_ += aData_[i];
    }
    
    checkSum_ = _serial.readByte();
    
    if((0xFF - ((id + data_) & 0xFF)) == checkSum_){
        if(id == kMsgIdMotorEncoder){
            _rEncoder.push_back(aData_[0]);
            _lEncoder.push_back(aData_[1]);
        }
        else if(id == kMsgIdGeoMagnetism){
            _gX.push_back(aData_[0]);
            _gY.push_back(aData_[1]);
            _gZ.push_back(aData_[2]);
        }
        else if(id == kMsgIdAcceleration){
            _aX.push_back(aData_[0]);
            _aY.push_back(aData_[1]);
            _aZ.push_back(aData_[2]);
        }
    }
    return;
}

/*! *******************************************************************
 *  @fn         sendData
 *  @brief      This function is send the Packet data.
 *
 *  @param[in]  id      :   Packet ID.
 *              value[] :   Packet data.
 *              length  :   Size of Packet data.
 *  @return     void
 *  @version    v1.04
 *  @date       10/17/2016  v1.04 [Refact] Serial Transmit.
 **********************************************************************/
void testApp::sendData(unsigned char id, short value[], size_t length){
    
    short value_ = 0;
    
    /*  [Note]: About Release Buffer of Vector Arrays.
     *    Vector Arrays is released out of scope.
     */
    vector <unsigned char> sendBuffer_;
    
    /**********************************
     * [Start bit].
     **********************************/
    sendBuffer_.push_back(DEF_SERIAL_PREFIX);
    
    /**********************************
     * [ID].
     **********************************/
    sendBuffer_.push_back(id);
    
    /**********************************
     * [Data].
     **********************************/
    
    /*  [Note]: About comparing signed and unsigned.
     *    C++ is alerted a Warning about comparing signed and unsigned.
     *    "length" variable is size_t type (unsigned int). And "i" variable
     *    into "for" is int (signed int).
     */
    for(unsigned int i = 0; i < length; ++i){
        sendBuffer_.push_back(value[i] >> 8);
        sendBuffer_.push_back(value[i] & DEF_SERIAL_MASK);
        value_ += value[i];
    }
    
    /**********************************
     * [CheckSum].
     **********************************/
    sendBuffer_.push_back(0xFF - ((id + value_) & 0xFF));
    
    /**********************************
     * [Serial].
     **********************************/
    _serial.writeBytes(&sendBuffer_[0], sendBuffer_.size());
    //Serial.write(&sendBuffer_.front(), sendBuffer_.size());
    
    return;
}

/*! *******************************************************************
 *  @fn         createCsv
 *  @brief      This function is a CSV file creating process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.01
 *  @date       10/06/2016 v1.01 [New func] Logging.
 **********************************************************************/
void testApp::createCsv(){
    
    unsigned int s_  = ofGetSeconds();
    unsigned int m_  = ofGetMinutes();
    unsigned int h_  = ofGetHours();
    unsigned int d_  = ofGetDay();
    unsigned int mo_ = ofGetMonth();
    unsigned int y_  = ofGetYear();

    _csv.setString(0, 0, DEF_CSV_TIMESTAMP);
    _csv.setString(0, 1, DEF_CSV_CDS);
    _csv.setString(0, 2, DEF_CSV_VOLTAGE);
    _csv.setString(0, 3, DEF_CSV_CURRENT);
    
    // Year(YYYY) + Month(MM) + Day(DD) +
    // Hour(HH) + Minute(MM) + Second(SS) + .csv
    _fileName = ofToString(y_, 0) + ofToString(mo_, 0) +
    ofToString(d_, 0) + ofToString(h_, 0) +
    ofToString(m_, 0) + ofToString(s_, 0) + ".csv";
    
    // save a file.
    _csv.saveFile(ofToDataPath(_fileName));
    
    return;
}

/*! *******************************************************************
 *  @fn         updateCsv
 *  @brief      This function is a CSV file updating process .
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.01
 *  @date       10/06/2016 v1.01 [New func] Logging.
 **********************************************************************/
void testApp::updateCsv(){
    
    unsigned int s_ = ofGetSeconds();
    unsigned int m_ = ofGetMinutes();
    unsigned int h_ = ofGetHours();
    
    string time_ = ofToString(h_, 0) + ":" +
    ofToString(m_, 0) + ":" +
    ofToString(s_, 0);
    
    
    /**************************************************
     * Time Stamp.
     **************************************************/
    _csv.setString((_row += 1), (_col = 0), time_);
    
    /**************************************************
     * Cds.
     **************************************************/
    _csv.setFloat(_row, (_col += 1), _cds);
    
    /**************************************************
     * Voltage.
     **************************************************/
    _csv.setInt(_row, (_col += 1), _voltage);
    
    /**************************************************
     * Current.
     **************************************************/
    _csv.setInt(_row, (_col += 1), _current);
    
    
    // save a file.
    _csv.saveFile(ofToDataPath(_fileName));
    
    return;
}

/*! *******************************************************************
 *  @fn         onAccelPushed
 *  @brief      This function is pushed the toggle button
 *              for Accelaration Sensor.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.05
 *  @date       10/17/2016 v1.05 [New func] Serial Receive.
 **********************************************************************/
void testApp::onAccelPushed(bool &value){
    return value ? (_isAccel = true) : (_isAccel = false);
}

/*! *******************************************************************
 *  @fn         onEncoderPushed
 *  @brief      This function is pushed the toggle button
 *              for Encoder Sensor.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.05
 *  @date       10/17/2016 v1.05 [New func] Serial Receive.
 **********************************************************************/
void testApp::onEncoderPushed(bool &value){
    return value ? (_isEncoder = true) : (_isEncoder = false);
}

/*! *******************************************************************
 *  @fn         onGeomagneticPushed
 *  @brief      This function is pushed the toggle button 
 *              for Geomagnetic Sensor.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.05
 *  @date       10/17/2016 v1.05 [New func] Serial Receive.
 **********************************************************************/
void testApp::onGeomagneticPushed(bool &value){
    return value ? (_isGeomagnetic = true) : (_isGeomagnetic = false);
}

/*! *******************************************************************
 *  @fn         onDrivePushed
 *  @brief      This function is pushed the toggle button for drive.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.06
 *  @date       10/03/2016 v1.00 Create On.
 *              10/07/2016 v1.02 [Bug fix] Released Vector.
 *              10/17/2016 v1.04 [Refact] Serial Transmit.
 *              11/08/2016 v1.06 [Refact] Common Message.
 **********************************************************************/
void testApp::onDrivePushed(bool &value){

    short data_[1] = {0};
    
    if(value){
        data_[0] = 1;
    }
    else{
        data_[0] = 0;
    }
    
    /*  [Note]: About an arraySize variable.
     *
     *    sizeof(value)   : Total byte number of array.
     *    sizeof(value[0]): 1 element size.
     *
     *    (Total byte number of array) / (1 element size) = array size
     */
    sendData(kMsgIdStartStatus,
             data_, sizeof(data_) / sizeof(data_[0]));
    
    
    return;
}

/*! *******************************************************************
 *  @fn         onSendDataPushed
 *  @brief      This function is pushed the toggle button for serial.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.00
 *  @date       10/03/2016 v1.00 Create On.
 **********************************************************************/
void testApp::onSendDataPushed(){
    return (_isSending = true);
}

/*! *******************************************************************
 *  @fn         onSensorPushed
 *  @brief      This function is pushed the toggle button for sensor.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.06
 *  @date       10/03/2016 v1.00 Create On.
 *              11/08/2016 v1.06 [Refact] Common Message.
 **********************************************************************/
void testApp::onSensorPushed(bool &value){

    short data_[1] = {0};

    if(value){
        _isSensor = true;
        data_[0]  = 1;
    }
    else{
        _isSensor = false;
        data_[0]  = 0;
    }

    sendData(kMsgIdSensingStatus,
             data_, sizeof(data_) / sizeof(data_[0]));

    return;
}

/*! *******************************************************************
 *  @fn         onLoggingPushed
 *  @brief      This function is changed the toggle button for Logging.
 *
 *  @param[in]  value   :   Toggle Status(1: ON, 0: OFF)
 *  @return     void
 *  @version    v1.01
 *  @date       10/06/2016 v1.01 [New func] Logging.
 **********************************************************************/
void testApp::onLoggingPushed(bool &value){

    if(value){
        _isLogging = true;
        createCsv();
    }
    else{
        _isLogging = false;
    }
}

/*! *******************************************************************
 *  @fn         onRightVelocityChanged
 *  @brief      This function is changed the slider for Right Velocity.
 *
 *  @param[in]  value   :   Slider Value
 *  @return     void
 *  @version    v1.00
 *  @date       10/03/2016 v1.00 Create On.
 **********************************************************************/
void testApp::onRightVelocityChanged(int &value){
    return (_rData = (short)value);
}

/*! *******************************************************************
 *  @fn         onLeftVelocityChanged
 *  @brief      This function is changed the slider for Left Velocity.
 *
 *  @param[in]  value   :   Slider Value
 *  @return     void
 *  @version    v1.00
 *  @date       10/03/2016 v1.00 Create On.
 **********************************************************************/
void testApp::onLeftVelocityChanged(int &value){
    return (_lData = (short)value);
}

/*! *******************************************************************
 *  @fn         setup [Default function]
 *  @brief      This function is the Initilize process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.01
 *  @date       10/03/2016 v1.00 Create On.
 *              10/06/2016 v1.01 [New func] Logging.
 **********************************************************************/
void testApp::setup(){

    /**************************************************
     * Common.
     **************************************************/
    ofBackground(ofColor::white);
    ofSetWindowTitle(DEF_GUI_WINDOW_TITLE);

    /**************************************************
     * Init.
     **************************************************/
    _isSending = false;
    _isSensor  = false;
    _isLogging = false;
    
    _row       = 0;
    _col       = 0;
    _cds       = 0;
    _voltage   = 0;
    _current   = 0;
    _rData     = 0;
    _lData     = 0;

    /**************************************************
     * Serial.
     **************************************************/
    _serial.setup(DEF_SERIAL_PORT, DEF_SERIAL_BAURATE);

    /**************************************************
     * Gui [Controlling].
     **************************************************/
    _gui.setup(DEF_GUI_CONTROLLING);
//    _gui.setSize(400, 20);

    _gui.add(_drive.setup(DEF_GUI_DRIVE, false));
    _gui.add(_send.setup(DEF_GUI_SEND));
    
    _gui.add(_rVelocity.setup(DEF_GUI_RIGHT_VELOCITY,
                              DEF_GUI_VELOCITY_INIT,
                              DEF_GUI_VELOCITY_MIN,
                              DEF_GUI_VELOCITY_MAX));
    
    _gui.add(_lVelocity.setup(DEF_GUI_LEFT_VELOCITY,
                              DEF_GUI_VELOCITY_INIT,
                              DEF_GUI_VELOCITY_MIN,
                              DEF_GUI_VELOCITY_MAX));
    
    _drive.addListener(this, &testApp::onDrivePushed);
    _send.addListener(this, &testApp::onSendDataPushed);

    _rVelocity.addListener(this, &testApp::onRightVelocityChanged);
    _lVelocity.addListener(this, &testApp::onLeftVelocityChanged);

    /**************************************************
     * Gui [Debug].
     **************************************************/
    _gui1.setup(DEF_GUI_DEBUG );
    _gui1.setPosition(230, 10);
    _gui1.add(_sensor.setup(DEF_GUI_SENSOR, false));
    _gui1.add(_logging.setup(DEF_GUI_LOGGING, false));
    
    _sensor.addListener(this, &testApp::onSensorPushed);
    _logging.addListener(this, &testApp::onLoggingPushed);

    /**************************************************
     * Gui [Visualization].
     **************************************************/
    _gui2.setup(DEF_GUI_VISUAL);
    _gui2.setPosition(450, 10);
    _gui2.add(_geomagnetic.setup(DEF_GUI_GEOMAGNETIC, false));
    _gui2.add(_encoder.setup(DEF_GUI_ENCODER, false));
    _gui2.add(_accel.setup(DEF_GUI_ACCEL, false));
    
    _geomagnetic.addListener(this, &testApp::onGeomagneticPushed);
    _encoder.addListener(this, &testApp::onEncoderPushed);
    _accel.addListener(this, &testApp::onAccelPushed);
    
    
    return;
}

/*! *******************************************************************
 *  @fn         update [Default function]
 *  @brief      This function is the Data Update process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.06
 *  @date       10/03/2016 v1.00 Create On.
 *              10/06/2016 v1.01 [New func] Logging.
 *              10/07/2016 v1.02 [Bug fix] Released Vector.
 *              10/12/2016 v1.03 [New func] Graph Display.
 *              10/17/2016 v1.04 [Refact] Serial Transmit.
 *              10/17/2016 v1.05 [New func] Serial Receive.
 *              11/08/2016 v1.06 [Refact] Common Message.
 **********************************************************************/
void testApp::update(){

    short data_[2] = {0, 0};
    
    /**************************************************
     * Serial.
     **************************************************/
    if(_isSending){
        _isSending = false;
        data_[0]   = _rData;
        data_[1]   = _lData;
        
        sendData(kMsgIdDriveVelocity,
                 data_, sizeof(data_) / sizeof(data_[0]));
    }

    if(_isSensor){
        while(_serial.available() > 0){
            if(_serial.readByte() == DEF_SERIAL_PREFIX){
                recieveData(_serial.readByte());
            }
        }
    }
    
    /**************************************************
     * Sensor Graph.
     **************************************************/
#if 0
    // **** For Debug. ****
     _lEncoder.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
     _rEncoder.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
    _gX.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
    _gY.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
    _gZ.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
    _aX.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
    _aY.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
    _aZ.push_back(ofRandom(DEF_GUI_R_MIN, DEF_GUI_R_MAX));
#endif

    if(_lEncoder.size() > 980){
        _lEncoder.erase(_lEncoder.begin());
    }
    
    if(_rEncoder.size() > 980){
        _rEncoder.erase(_rEncoder.begin());
    }
    
    if(_gX.size() > 980){
        _gX.erase(_gX.begin());
    }
        
    if(_gY.size() > 980){
        _gY.erase(_gY.begin());
    }
        
    if(_gZ.size() > 980){
        _gZ.erase(_gZ.begin());
    }
    
    if(_aX.size() > 980){
        _aX.erase(_aX.begin());
    }
    
    if(_aY.size() > 980){
        _aY.erase(_aY.begin());
    }
    
    if(_aZ.size() > 980){
        _aZ.erase(_aZ.begin());
    }

    /**************************************************
     * Logging.
     **************************************************/
    if(_isLogging){
        updateCsv();
    }
    else{
        _row = 0;
        _col = 0;
    }

    return;
}

/*! *******************************************************************
 *  @fn         draw [Default function]
 *  @brief      This function is the Draw process.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.05
 *  @date       10/03/2016 v1.00 Create On.
 *              10/06/2016 v1.01 [New func] Logging.
 *              10/12/2016 v1.03 [New func] Graph Display.
 *              10/17/2016 v1.05 [New func] Serial Receive.
 **********************************************************************/
void testApp::draw(){

    float lEncoderT_       = 0.;
    float rEncoderT_       = 0.;
    float gXT_, gYT_, gZT_ = 0.;
    float aXT_, aYT_, aZT_ = 0.;
    
    
    ofBackground(ofColor::black);

    /**************************************************
     * Graph Frame.
     **************************************************/
    ofSetLineWidth(2);
    ofSetColor(ofColor::white);
    
    /*  [Note]: About the line of square for a graph.
     *      Line definitions are the following figure and comment.
     *
     *            (3)
     *     ------------------
     *    |                  |
     * (1)|                  |(2)
     *    |                  |
     *     ------------------
     *            (4)
     */
    
    // (1) Left Vertical axis
    ofLine(20, (ofGetHeight() / 2) + DEF_GUI_G_MAX,
           20, (ofGetHeight() / 2) - DEF_GUI_G_MAX);
    
    // (2) Right Vertical axis
    ofLine(1000, (ofGetHeight() / 2) + DEF_GUI_G_MAX,
           1000, (ofGetHeight() / 2) - DEF_GUI_G_MAX);

    // (3) Up Horizontal axis
    ofLine(20, (ofGetHeight() / 2) - DEF_GUI_G_MAX,
           1000, (ofGetHeight() / 2) - DEF_GUI_G_MAX);

    // (4) Down Horizontal axis
    ofLine(20, (ofGetHeight() / 2) + DEF_GUI_G_MAX,
           1000, (ofGetHeight() / 2) + DEF_GUI_G_MAX);

    /**************************************************
     * Graph Description.
     **************************************************/
    ofSetColor(ofColor::white);
    ofDrawBitmapString(DEF_GUI_LEFT_ENCODER, 20, 620);
    ofSetLineWidth(2);
    ofLine(20, 623, 120, 623);

    ofSetColor(ofColor::red);
    ofDrawBitmapString(DEF_GUI_RIGHT_ENCODER, 20, 640);
    ofSetLineWidth(2);
    ofLine(20, 643, 120, 643);

    ofSetColor(ofColor::yellow);
    ofDrawBitmapString(DEF_GUI_GEO_X, 150, 620);
    ofSetLineWidth(2);
    ofLine(150, 623, 250, 623);

    ofSetColor(ofColor::gray);
    ofDrawBitmapString(DEF_GUI_GEO_Y, 150, 640);
    ofSetLineWidth(2);
    ofLine(150, 643, 250, 643);

    ofSetColor(ofColor::blue);
    ofDrawBitmapString(DEF_GUI_GEO_Z, 150, 660);
    ofSetLineWidth(2);
    ofLine(150, 663, 250, 663);

    ofSetColor(ofColor::pink);
    ofDrawBitmapString(DEF_GUI_ACCEL_X, 290, 620);
    ofSetLineWidth(2);
    ofLine(290, 623, 400, 623);

    ofSetColor(ofColor::green);
    ofDrawBitmapString(DEF_GUI_ACCEL_Y, 290, 640);
    ofSetLineWidth(2);
    ofLine(290, 643, 400, 643);

    ofSetColor(ofColor::yellow);
    ofDrawBitmapString(DEF_GUI_ACCEL_Z, 290, 660);
    ofSetLineWidth(2);
    ofLine(290, 663, 400, 663);

    
    if(_isSensor){

        /**************************************************
         * Encoder.
         **************************************************/
        if(_isEncoder){
            ofNoFill();
            ofSetColor(ofColor::white);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _lEncoder.size(); ++i){
                lEncoderT_ = ofMap(_lEncoder[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                                   DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_lEncoder.size() - i) + 20,
                          lEncoderT_ + ofGetHeight() / 2);
            }
            ofEndShape();
            
            ofNoFill();
            ofSetColor(ofColor::red);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _rEncoder.size(); ++i){
                rEncoderT_ = ofMap(_rEncoder[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                                   DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_rEncoder.size() - i) + 20,
                          rEncoderT_ + ofGetHeight() / 2);
            }
            ofEndShape();
        }

        /**************************************************
         * Geomagnetic Sensor.
         **************************************************/
        if(_isGeomagnetic){
            ofNoFill();
            ofSetColor(ofColor::yellow);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _gX.size(); ++i){
                gXT_ = ofMap(_gX[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                             DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_gX.size() - i) + 20,
                         gXT_ + ofGetHeight() / 2);
            }
            ofEndShape();
            
            ofNoFill();
            ofSetColor(ofColor::gray);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _gY.size(); ++i){
                gYT_ = ofMap(_gY[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                             DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_gY.size() - i) + 20,
                         gYT_ + ofGetHeight() / 2);
            }
            ofEndShape();
            
            ofNoFill();
            ofSetColor(ofColor::blue);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _gZ.size(); ++i){
                gZT_ = ofMap(_gZ[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                             DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_gZ.size() - i) + 20,
                         gZT_ + ofGetHeight() / 2);
            }
            ofEndShape();
        }
        /**************************************************
         * Accelaration Sensor.
         **************************************************/
        if(_isAccel){
            ofNoFill();
            ofSetColor(ofColor::pink);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _aX.size(); ++i){
                aXT_ = ofMap(_aX[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                             DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_aX.size() - i) + 20,
                         aXT_ + ofGetHeight() / 2);
            }
            ofEndShape();
            
            ofNoFill();
            ofSetColor(ofColor::green);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _aY.size(); ++i){
                aYT_ = ofMap(_aY[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                             DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_aY.size() - i) + 20,
                         aYT_ + ofGetHeight() / 2);
            }
            ofEndShape();
            
            ofNoFill();
            ofSetColor(ofColor::yellow);
            ofSetLineWidth(1.5);
            
            ofBeginShape();
            for(int i = 0; i < _aZ.size(); ++i){
                aZT_ = ofMap(_aZ[i], DEF_GUI_R_MIN, DEF_GUI_R_MAX,
                             DEF_GUI_G_MIN, DEF_GUI_G_MAX);
                
                ofVertex((_aZ.size() - i) + 20,
                         aZT_ + ofGetHeight() / 2);
            }
            ofEndShape();
        }

        
    }

    /**************************************************
     * GUI.
     **************************************************/
    _gui.draw();
    _gui1.draw();
    _gui2.draw();
    
    return;
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
