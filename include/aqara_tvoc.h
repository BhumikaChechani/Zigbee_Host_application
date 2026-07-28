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

typedef struct {
    uint16_t shortAddr;
    uint8_t  endpoint;
    uint8_t  ieee[8];
    bool     hasIeee;
    double   lastSeen;
} AQARA_TVOC_T;

extern AQARA_TVOC_T g_aqaraTvocs[MAX_AQARA_TVOC];
extern int g_numAqaraTvocs;

// Initialization & Thread
void AqaraTvoc_Init(void);
void AqaraTvoc_Start(void);

// Management
void AqaraTvoc_Discover(uint16_t addr, uint8_t ep);
bool AqaraTvoc_IsKnown(uint16_t addr);
void AqaraTvoc_UpdateSeen(uint16_t addr);
void AqaraTvoc_ReadEnvironment(uint16_t addr);

// Processing incoming messages
void AqaraTvoc_PostAf(uint16_t addr, const AF_MSG_T *af);

#endif // AQARA_TVOC_H
