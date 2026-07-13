#ifndef VIBRATION_SENSOR_H
#define VIBRATION_SENSOR_H

#include "znp_host.h"
#include "sensor_common.h"
#include <stdbool.h>
#include <stdint.h>

#define MAX_VIBRATION_SENSORS 16

typedef struct
{
    uint16_t shortAddr;
    uint8_t endpoint;
    uint8_t ieee[8];
    bool hasIeee;
    double lastSeen;
    int zoneId;
    bool configured;
    bool isVibrating;
    double lastVibrationTime;
    bool isMoving;
    double lastMovementTime;
    uint16_t lastZoneStatus;
} VIBRATION_SENSOR_T;

extern VIBRATION_SENSOR_T g_vibrationSensors[MAX_VIBRATION_SENSORS];
extern int g_numVibrationSensors;

void VibrationSensor_Init( void );
void VibrationSensor_Start( void );
void VibrationSensor_Discover( uint16_t shortAddr_, uint8_t endpoint_ );
void VibrationSensor_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ );
void VibrationSensor_Setup( uint16_t shortAddr_ );
void VibrationSensor_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ );
void VibrationSensor_HandleStatus( uint16_t shortAddr_, uint16_t zoneStatus_, uint8_t zoneId_ );
void VibrationSensor_PrintStatus( void );
bool VibrationSensor_IsKnown( uint16_t shortAddr_ );
void VibrationSensor_UpdateSeen( uint16_t shortAddr_ );
void VibrationSensor_DiscoverAllActiveEp( void );
void VibrationSensor_PollAll( void );
void VibrationSensor_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ );

void VibrationSensor_SetSensitivity( uint16_t shortAddr_, uint8_t level_ );
void VibrationSensor_ReadEnvironment( uint16_t shortAddr_ );

#endif // VIBRATION_SENSOR_H
