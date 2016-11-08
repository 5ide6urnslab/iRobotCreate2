#ifndef MESSAGE_ID_H
#define MESSAGE_ID_H

#ifdef __cplusplus
namespace MessageIds {
#endif // __cplusplus

enum MessageId {
  kMsgIdInvalid = -1,

  /*****************************************************************
   * [Status control messages]
   *  Host --> control --> Target
   *****************************************************************/
  kMsgIdStartStatus,
  kMsgIdSensingStatus,
  /*****************************************************************
   * [Actuator control messages]
   *  Host --> control --> Target
   *****************************************************************/
  kMsgIdDriveVelocity,

  /*****************************************************************
   * [Notify messages]
   *  Host <-- notify <-- Target
   *****************************************************************/
  kMsgIdInformation,
  kMsgIdGyro,
  kMsgIdGeoMagnetism,
  kMsgIdAcceleration,
  kMsgIdCds,
  kMsgIdPowerSupply,
  kMsgIdMotorEncoder
};

#ifdef __cplusplus
}
typedef MessageIds::MessageId MessageId;
#else  // __cplusplus

typedef enum MessageId MessageId;
#endif // __cplusplus

#endif // MESSAGE_ID_H
