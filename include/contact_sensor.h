#ifndef CONTACT_SENSOR_H
#define CONTACT_SENSOR_H

#include "znp_host.h"
#include "sensor_common.h"
#include <stdbool.h>
#include <stdint.h>

#define MAX_CONTACT_SENSORS 16

typedef struct
{
    uint16_t shortAddr;
    uint8_t endpoint;
    uint8_t ieee[8];
    bool hasIeee;
    double lastSeen;
    int zoneId;
    bool configured;        ///< True once bind + CIE write succeeded (verified).
    bool isOpen;
    bool isTampered;
    uint8_t setupRetries;   ///< Failed setup attempts (retry cap).
    double lastSetupAttempt;///< Timestamp of the last setup attempt (retry pacing).
    double lastStatusTime;  ///< Timestamp of the last zone status update received.
    double lastOpenedTime;  ///< Timestamp of the last CLOSED->OPEN transition.
    double lastRefreshReq;  ///< Timestamp of the last active status refresh request.
} CONTACT_SENSOR_T;

extern CONTACT_SENSOR_T g_contactSensors[MAX_CONTACT_SENSORS];
extern int g_numContactSensors;

void ContactSensor_Init( void );
void ContactSensor_Start( void );
void ContactSensor_Discover( uint16_t shortAddr_, uint8_t endpoint_ );
void ContactSensor_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ );
void ContactSensor_Setup( uint16_t shortAddr_ );
void ContactSensor_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ );
void ContactSensor_HandleStatus( uint16_t shortAddr_, uint16_t zoneStatus_, uint8_t zoneId_ );
void ContactSensor_PrintStatus( void );
bool ContactSensor_IsKnown( uint16_t shortAddr_ );
void ContactSensor_UpdateSeen( uint16_t shortAddr_ );
void ContactSensor_DiscoverAllActiveEp( void );
void ContactSensor_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ );
void ContactSensor_ReadEnvironment( uint16_t shortAddr_ );

/// @brief  Queue an active IAS zone-status read for one sensor (non-blocking).
void ContactSensor_PostRefresh( uint16_t shortAddr_ );

/// @brief  Queue a zone-status refresh for every sensor whose last status
///         update is older than @p maxAgeSeconds_ (self-rate-limited).
///         Call from the use-case thread before trusting a stored door state.
void ContactSensor_RefreshIfStale( double maxAgeSeconds_ );

#endif // CONTACT_SENSOR_H
