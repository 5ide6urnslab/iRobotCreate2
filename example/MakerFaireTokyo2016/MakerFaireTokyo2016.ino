
/*************************************************************************
 * File Name          : MakerFaireTokyo2016
 * Author             : Show Kawabata(5ide6urns lab)
 * Version            : v1.06
 * Date               : 09/01/2016
 * Parts required     : Arduino MEGA 2560 R3 , XBee ZB Series 2(Digi Internatinal),
 *                      Cds 5mm Sensor(Macron Internatinal), INA226(Texas Instruments)
 * Description        :
 *
 * License            : Released under the MIT license.
 *                      http://opensource.org/licenses/mit-license.php
 *
 * Copyright          : Copyright (C) 2016 5ide6urns lab All right reserved.
 * History            : 07/28/2016 v1.00 Show Kawabata Create On.
 *                      08/17/2016 v1.01 Show Kawabata [Bug Fix] Serial Data Missed.
 *                      08/19/2016 v1.02 Show Kawabata [New func] Straight Drive.
 *                      08/26/2016 v1.03 Show Kawabata [Refactoring] Data Coupling function.
 *                      08/26/2016 v1.04 Show Kawabata [Refactoring] Serial Sending way.
 *                      08/30/2016 v1.05 Show Kawabata [Refactoring] uClibc++ for Arduino.
 *                      09/01/2016 v1.06 Show Kawabata [Bug Fix] Compare signed and unsigned.
 *************************************************************************/

#include <Arduino.h>                         // For PlatformIO Input Completion.
#include <iRobotCreate2.h>
#include <Ina226.h>
#include "create2_message_id.h"
#include <StandardCplusplus.h>               // uClibc++ for Arduino.
#include <vector>

using namespace Create2MessageIds;
using namespace std;

#define DEBUG
#define SERIAL_BAURATE  115200

/**********************************
 * Receive Data Structure.
 **********************************/
typedef struct{
  byte _id;
  int  _driveR;
  int  _driveL;
  int  _dStatus;
  int  _sStatus;
} PACKET;

PACKET        _pt;

/**********************************
 * iRobot.
 **********************************/
iRobotCreate2 _roomba = iRobotCreate2();

/**********************************
 * INA226.
 **********************************/
Ina226 _ina226 = Ina226();

/**********************************
 * Cds.
 **********************************/
const int _cdsPin       = 0;

/**********************************
 * Protype Declaration.
 **********************************/

/*  [Note]: About the Protype Declaration.
 *    The Protype Declaration is inserted after the Preprocessor(#define etc)
 *    by the Arduino IDE. In other words, Before the data of typedef Declaration,
 *    There is the Protype Declaration used the data of typedef.
 *    In the case of the Protype Declaration is not have, It is compile error.
 */
bool recieveData();
void sendData(int id, int value[], size_t length);
void getSensorData();
void sendCdsSensor();
void ina226Init();
void sendIna226();
void dataCoupling(int msb, int lsb, int* value);


/*! *******************************************************************
 *  @fn         setup
 *  @brief      This function is the Initial Process of Arduino.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       07/28/2016  v1.00:  Create on.
 *              08/19/2016  v1.02:  [New func] Straight Drive.
 **********************************************************************/
void setup() {

  /**********************************
   * Roomba Init Process.
   **********************************/
  _roomba.start();
  _roomba.full();
  delay(1000);

  /**********************************
   * Serial Init Process.
   **********************************/
  Serial.begin(SERIAL_BAURATE);

#ifdef DEBUG
  Serial2.begin(SERIAL_BAURATE);
  Serial2.println("Serial Start");
#endif

  /**********************************
   * Variable Init Process.
   **********************************/
  _pt._driveL  = 150;
  _pt._driveR  = -150;
  _pt._dStatus = 0;
  _pt._sStatus = 0;
  _pt._id      = 1;

  _roomba.drive(_pt._driveR, _pt._driveL);

  /**********************************
   * INA226 Init Process.
   **********************************/
  ina226Init();

  return;
}

/*! *******************************************************************
 *  @fn         loop
 *  @brief      This function is the Loop Process of Arduino.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.02
 *  @date       07/28/2016  v1.00:  Create on.
 *              08/19/2016  v1.02:  [New func] Straight Drive.
 **********************************************************************/
void loop() {

  /**********************************
   * Receive Data from iRobot.
   **********************************/
  if(_pt._sStatus){
    getSensorData();
  }

  /**********************************
   * Receive Data from ROS.
   **********************************/
  bool a_ = recieveData();

  if(!a_){
    return;
  }

  /**********************************
   * Control Status.(Start/Stop)
   **********************************/
  if(_pt._id == kCreate2MsgIdStartStatus){

    /**********************************
     * Stop Status.
     **********************************/
    if(!_pt._dStatus){
      _roomba.stop();

#ifdef DEBUG
      Serial2.println("Stop Status");
#endif

    }
    /**********************************
     * Start Status.
     **********************************/
    else{
      _roomba.start();
      _roomba.full();
      delay(1000);
    }
  }
  /**********************************
   * Drive.
   **********************************/
  else if (_pt._id == kCreate2MsgIdDriveVelocity){

    _roomba.drive(_pt._driveR, _pt._driveL);
  }
  else{
    // Nothing
  }

  return;
}

/*! *******************************************************************
 *  @fn         dataCoupling
 *  @brief      This function is coupled Data formed by
 *              coupling two pcs of data by Serial Communication.
 *
 *  @param[in]  msb       :   Most Signficant Bit for a data(2 byte).
 *              lsb       :   Least Signficant Bit for a data(2 byte)
 *  @param[out] *value    :   Data formed by coupling two pcs of data.
 *  @return     void
 *  @version    v1.03
 *  @date       07/28/2016  v1.01:  [Bug Fix] Serial Data Missed.
 *              08/26/2016  v1.03:  [Refactoring] Data Coupling function.
 **********************************************************************/
void dataCoupling(int msb, int lsb, int* value){

  *value = (msb << 8) | lsb;

  return;
}

/*! *******************************************************************
 *  @fn         recieveData
 *  @brief      This function is the Packet Reciever.
 *
 *  @param[in]  void
 *  @return     result   the Recieve Status.(SUCCESS:true / FAILURE:false)
 *  @version    v1.03
 *  @date       07/28/2016  v1.00:  Create on.
 *              08/17/2016  v1.01:  [Bug Fix] Serial Data Missed.
 *              08/26/2016  v1.03:  [Refactoring] Data Coupling function.
 **********************************************************************/
boolean recieveData(){

  int result_   = false;
  int id_       = 0;
  int driveR_   = 0;
  int driveL_   = 0;
  int dStatus_  = 0;
  int sStatus_  = 0;
  int checkSum_ = 0;
  int msb_      = 0;
  int lsb_      = 0;


  /*  [Note]: about the Communication Packet.
   *    The ID and Data Length, Value is defined.
   *
   *    [Definition]:
   *    ----------------------------------------------------------------------
   *     Start Bit    ID    Value      Description
   *    ----------------------------------------------------------------------
   *     $            (*2)  0          [Read only]  iROBOT CONTROL STATUS(Stop)
   *     $            (*2)  1          [Read only]  iROBOT CONTROL STATUS(Start)
   *     $            (*2)  0          [Read only]  SENSOR STATUS(Stop)
   *     $            (*2)  1          [Read only]  SENSOR STATUS(Start)
   *     $            (*2)  0 - 255    [Read only]  (*1) DRIVE CONTROL(Right)
   *     $            (*2)  0 - 255    [Read only]  (*1) DRIVE CONTROL(Left)
   *     $            (*2)  0 - 1023   [Write only] CDS SENSOR VALUE
   *     $            (*2)             [Write only] INA226 SENSOR VALUE
   *
   *    The Message is send every 1 byte by char type.
   *    The Message length is checked the Data Length.
   *
   *    (*1) In the case of ID "0x02", Right and Left Drive are send once.
   *    (*2) A Packet ID is refered "create2_message_id.h".
   *
   */

  if(Serial.available() > 0){

    /**********************************
     * [Start bit].
     **********************************/
    delay(10);
    if(Serial.read() == 0x24){

     /**********************************
      * [ID].
      **********************************/
     delay(10);
     id_ = Serial.read();

#ifdef DEBUG
      Serial2.print("ID: ");
      Serial2.println(id_);
#endif

      /**********************************
       * [Data] Drive Velocity.
       **********************************/
      if(id_ == kCreate2MsgIdDriveVelocity){

         /**********************************
          * Right Motor Velocity.
          **********************************/
         delay(10);
         msb_ = Serial.read();

         delay(10);
         lsb_ = Serial.read();

         dataCoupling(msb_, lsb_, &driveR_);

#ifdef DEBUG
         Serial2.print("Right Drive Value: ");
         Serial2.println(driveR_);
#endif

         /**********************************
          * Left Motor Velocity.
          **********************************/
         delay(10);
         msb_ = Serial.read();

         delay(10);
         lsb_ = Serial.read();

         dataCoupling(msb_, lsb_, &driveL_);

#ifdef DEBUG
         Serial2.print("Left Drive Value: ");
         Serial2.println(driveL_);
#endif

         delay(10);
         checkSum_ = Serial.read();

#ifdef DEBUG
         Serial2.print("Drive CheckSum: ");
         Serial2.println(checkSum_);
#endif

         if((0xFF - ((id_ + driveR_ + driveL_) & 0xFF)) == checkSum_){
             _pt._id     = id_;
             _pt._driveR = driveR_;
             _pt._driveL = driveL_;
             result_     = true;
         }
       }
       /**********************************
        * [Data] Drive Status.
        **********************************/
       else if(id_ == kCreate2MsgIdStartStatus){

         delay(10);
         msb_ = Serial.read();

         delay(10);
         lsb_ = Serial.read();

         dataCoupling(msb_, lsb_, &dStatus_);

#ifdef DEBUG
         Serial2.print("Drive Status: ");
         Serial2.println(dStatus_);
#endif

          delay(10);
          checkSum_ = Serial.read();

#ifdef DEBUG
          Serial2.print("Drive Status CheckSum: ");
          Serial2.println(checkSum_);
#endif

          if((0xFF - ((id_ + dStatus_) & 0xFF)) == checkSum_){
              _pt._id      = id_;
              _pt._dStatus = dStatus_;
              result_      = true;
          }
       }
       /**********************************
        * [Data] Sensor Status.
        **********************************/
       else if(id_ == kCreate2MsgIdSensingStatus){

         delay(10);
         msb_ = Serial.read();

         delay(10);
         lsb_ = Serial.read();

         dataCoupling(msb_, lsb_, &sStatus_);

#ifdef DEBUG
         Serial2.print("Sensor Status: ");
         Serial2.println(sStatus_);
#endif

         delay(10);
         checkSum_ = Serial.read();

#ifdef DEBUG
         Serial2.print("Sensor Status CheckSum: ");
         Serial2.println(checkSum_);
#endif

         if((0xFF - ((id_ + sStatus_) & 0xFF)) == checkSum_){
            _pt._id      = id_;
            _pt._sStatus = sStatus_;
            result_      = true;
         }
       }
    }
  }

  return result_;
}

/*! *******************************************************************
 *  @fn         sendData
 *  @brief      This function is send the Packet data.
 *
 *  @param[in]  id      :   Packet ID.
 *              value[] :   Packet data.
 *              length  :   Size of Packet data.
 *  @return     void
 *  @version    v1.06
 *  @date       07/28/2016  v1.00:  Create on.
 *              08/17/2016  v1.01:  [Bug Fix] Serial Data missed.
 *              08/26/2016  v1.04:  [Refactoring] Serial Sending way.
 *              08/26/2016  v1.05:  [Refactoring] uClibc++ for Arduino.
 *              09/01/2016  v1.06:  [Bug Fix] Compare signed and unsigned.
 **********************************************************************/
void sendData(int id, int value[], size_t length){

  int                    value_       = 0;
  vector <unsigned char> sendBuffer_;

  /**********************************
   * [Start bit].
   **********************************/
  sendBuffer_.push_back(0x24);

#ifdef DEBUG
    Serial2.print("Send Start Bit: ");
    Serial2.println(sendBuffer_[0]);
#endif

  /**********************************
   * [ID].
   **********************************/
  sendBuffer_.push_back(id);

#ifdef DEBUG
    Serial2.print("Send ID: ");
    Serial2.println(sendBuffer_[1]);
#endif

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
    sendBuffer_.push_back(value[i] & B11111111);
    value_ += value[i];
  }

  /**********************************
   * [CheckSum].
   **********************************/
  sendBuffer_.push_back(0xFF - ((id + value_) & 0xFF));

#ifdef DEBUG
    Serial2.print("Send CheckSum: ");
    Serial2.println(sendBuffer_[sendBuffer_.size() - 1]);
#endif

  /**********************************
   * [Serial].
   **********************************/
  //Serial.write(&sendBuffer_[0], sendBuffer_.size());
  Serial.write(&sendBuffer_.front(), sendBuffer_.size());

  return;
}

/*! *******************************************************************
 *  @fn         getSensorData
 *  @brief      This function is get iRobot and External Sensors data.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       07/28/2016  v1.00:  Create on.
 **********************************************************************/
void getSensorData(){

  /**********************************
   * Cds Sensor.
   **********************************/
  sendCdsSensor();

  /**********************************
   * INA226 Sensor.
   **********************************/
  delay(10);
  sendIna226();

  return;
}

/*! *******************************************************************
 *  @fn         sendCdsSensor
 *  @brief      This function is send a value of Cds Sensor.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       07/28/2016  v1.00:  Create on.
 **********************************************************************/
void sendCdsSensor(){

  int cds_[1] = {0};
  cds_[0] = analogRead(_cdsPin) / 4;

  /*  [Note]: About an arraySize variable.
   *
   *    sizeof(value)   : Total byte number of array.
   *    sizeof(value[0]): 1 element size.
   *
   *    (Total byte number of array) / (1 element size) = array size
   */
  sendData(kCreate2MsgIdCds, cds_, sizeof(cds_) / sizeof(cds_[0]));

  return;

}

/*! *******************************************************************
 *  @fn         ina226Init
 *  @brief      This function is the Initilaze Process of INA226.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       07/28/2016  v1.00:  Create on.
 **********************************************************************/
void ina226Init(){

  _ina226.configration(INA226_AVERAGES_64, INA226_VBUS_CT_2116US,
                       INA226_VSHUNT_CT_2116US, INA226_MODE_SHUNT_BUS_CONTINUOUS);

  _ina226.calibration();

  return;
}

/*! *******************************************************************
 *  @fn         sendIna226
 *  @brief      This function is send a value of INA226.
 *
 *  @param[in]  void
 *  @return     void
 *  @version    v1.00
 *  @date       07/28/2016  v1.00:  Create on.
 **********************************************************************/
void sendIna226(){

  short bv_     = 0;
  short sv_     = 0;
  short c_      = 0;

  /*  [Note]: About the "bvc_ array".
   *    [0] = Bus Voltage
   *    [1] = Current
   */
  int   bvc_[2] = {0, 0};

  _ina226.readIna226(INA226_REGISTER_BUS, &bv_);

  /*  [Note]: Abouy Bus Voltage.
   *    You should refer ina226.pdf, CONFIGURE/MEASURE/CALCULATE EXAMPLE.
   *    LSB = 1.25mV.
   */
  bvc_[0] = (int)bv_;
  bvc_[0] *= 0.00125;

  /*  [Note]: Abouy Power.
   *    You should refer ina226.pdf, CONFIGURE/MEASURE/CALCULATE EXAMPLE.
   *    LSB = 25mW.
   */
  _ina226.readIna226(INA226_REGISTER_SHUNT, &sv_);

  // ouput "mA" value.
  _ina226.readIna226(INA226_REGISTER_CURRENT, &c_);

  bvc_[1] = (int)c_;

  /*  [Note]: About an arraySize variable.
   *
   *    sizeof(value)   : Total byte number of array.
   *    sizeof(value[0]): 1 element size.
   *
   *    (Total byte number of array) / (1 element size) = array size
   */
  sendData(kCreate2MsgIdPowerSupply, bvc_, sizeof(bvc_) / sizeof(bvc_[0]));

  return;
}
