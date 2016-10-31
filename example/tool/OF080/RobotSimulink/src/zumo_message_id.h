#ifndef ZUMO_MESSAGE_ID_H
#define ZUMO_MESSAGE_ID_H

#ifdef __cplusplus
namespace ZumoMessageIds {
#endif // __cplusplus

enum ZumoMessageId {
  kZumoMsgIdInvalid = -1,
  kZumoMsgIdStartStatus,
  kZumoMsgIdSensingStatus,
  kZumoMsgIdDriveVelocity,
  kZumoMsgIdInformation,
  kZumoMsgIdColor,
  kZumoMsgIdGyro,
  kZumoMsgIdGeoMagnetism,
  kZumoMsgIdCds,
  kZumoMsgIdPowerSupply,
  kZumoMsgIdMotorEncoder
};

#ifdef __cplusplus
}
typedef ZumoMessageIds::ZumoMessageId ZumoMessageId;
#else  // __cplusplus

typedef enum ZumoMessageId ZumoMessageId;
#endif // __cplusplus

#endif // ZUMO_MESSAGE_ID_H
