#ifndef CREATE2_MESSAGE_ID_H
#define CREATE2_MESSAGE_ID_H

#ifdef __cplusplus
namespace Create2MessageIds {
#endif // __cplusplus

enum Create2MessageId {
  kCreate2MsgIdInvalid = -1,
  kCreate2MsgIdStartStatus, // 00
  kCreate2MsgIdSensingStatus,
  kCreate2MsgIdDriveVelocity,
  kCreate2MsgIdInformation,

  /*****************************************************************
   * [iRobot Sensor Packet ID].
   *  Serial Message IDs for iRobot is matched iRobot Sensor Packet
   *  IDs(from 7 to 58). [Ref]: Open Interface Spec.
   *****************************************************************/
  kCreate2MsgIdAngle = 20,

  /*****************************************************************
   * [External Sensor Packet ID].
   *****************************************************************/
  kCreate2MsgIdColor = 59,
  kCreate2MsgIdGyro,
  kCreate2MsgIdCds,
  kCreate2MsgIdPowerSupply,

};

#ifdef __cplusplus
}
typedef Create2MessageIds::Create2MessageId Create2MessageId;
#else  // __cplusplus

typedef enum Create2MessageId Create2MessageId;
#endif // __cplusplus

#endif // CREATE2_MESSAGE_ID_H
