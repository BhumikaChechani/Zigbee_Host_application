#ifndef AQARA_TVOC_H
#define AQARA_TVOC_H

#include "znp_host.h"
#include "sensor_common.h"

// ---------------------------------------------------------------------------
// Aqara TVOC (AAQS-S01) Clusters
// ---------------------------------------------------------------------------
#define AQARA_TVOC_TEMP_CLUSTER     0x0402U
#define AQARA_TVOC_HUM_CLUSTER      0x0405U
#define AQARA_TVOC_ANALOG_CLUSTER   0x000CU  // TVOC is reported here (genAnalogInput)
#define AQARA_TVOC_POWER_CLUSTER    0x0001U

#define MAX_AQARA_TVOC 16

typedef enum {
    TVOC_AQ_UNKNOWN = 0,
    TVOC_AQ_EXCELLENT,
    TVOC_AQ_GOOD,
    TVOC_AQ_MODERATE,
    TVOC_AQ_POOR,
    TVOC_AQ_UNHEALTHY
} TVOC_AQ_STATE_T;

typedef struct {
    uint16_t shortAddr;
    uint8_t  endpoint;
    uint8_t  ieee[8];
    bool     hasIeee;
    double   lastSeen;
    TVOC_AQ_STATE_T lastAirQuality;
    float    lastTemp;
    double   lastTempTime;
    float    lastHum;
    double   lastHumTime;
    float    lastTvoc;
    double   lastTvocTime;
    uint8_t  lastBatt;
    double   lastBattTime;
} AQARA_TVOC_T;

extern AQARA_TVOC_T g_aqaraTvocs[MAX_AQARA_TVOC];
extern int g_numAqaraTvocs;

// Initialization & Thread
void AqaraTvoc_Init(void);
void AqaraTvoc_Start(void);

// Management
void AqaraTvoc_Discover(uint16_t addr, uint8_t ep);
void AqaraTvoc_UpdateIeee(uint16_t shortAddr_, const uint8_t *ieee_);
bool AqaraTvoc_IsKnown(uint16_t addr);
void AqaraTvoc_UpdateSeen(uint16_t addr);
void AqaraTvoc_ReadEnvironment(uint16_t addr);
void AqaraTvoc_PrintStatus(void);

// Processing incoming messages
void AqaraTvoc_PostAf(uint16_t addr, const AF_MSG_T *af);

#endif // AQARA_TVOC_H
