#include "aqara_tvoc.h"
#include "config.h"
#include "logger.h"
#include "msg_queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

#if ENABLE_AQARA_TVOC

AQARA_TVOC_T g_aqaraTvocs[MAX_AQARA_TVOC];
int g_numAqaraTvocs = 0;

static pthread_t s_thread;
static MSG_QUEUE_T s_inbox;
extern pthread_mutex_t g_deviceMutex;

static void HandleAf(const AF_MSG_T *af)
{
    if (af->dataLen < 3) return;
    uint8_t fc = af->data[0];
    int hdrLen = (fc & 0x04) ? 5 : 3;
    if (af->dataLen < hdrLen) return;
    uint8_t cmdId = af->data[hdrLen-1];
    const uint8_t *zcl = &af->data[hdrLen];
    int zclLen = af->dataLen - hdrLen;

    if (cmdId == 0x0A || cmdId == 0x01) { // Report or Read Rsp
        if (zclLen >= 4) {
            uint16_t attr = zcl[0] | (zcl[1] << 8);
            uint8_t dataType = zcl[2];
            
            // Temperature
            if (af->clusterId == AQARA_TVOC_TEMP_CLUSTER && attr == 0x0000 && dataType == 0x29 && zclLen >= 5) {
                int16_t tempRaw = zcl[3] | (zcl[4] << 8);
                float tempC = tempRaw / 100.0f;
                LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;36mTemperature: %.2f°C\033[0m\n", tempC);
            }
            // Humidity
            else if (af->clusterId == AQARA_TVOC_HUM_CLUSTER && attr == 0x0000 && dataType == 0x21 && zclLen >= 5) {
                uint16_t humRaw = zcl[3] | (zcl[4] << 8);
                float humPercent = humRaw / 100.0f;
                LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;36mHumidity: %.2f%%\033[0m\n", humPercent);
            }
            // TVOC (genAnalogInput, presentValue is attr 0x0055, type single precision float 0x39)
            else if (af->clusterId == AQARA_TVOC_ANALOG_CLUSTER && attr == 0x0055 && dataType == 0x39 && zclLen >= 7) {
                float tvoc = 0.0f;
                memcpy(&tvoc, &zcl[3], 4);
                LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;35mTVOC: %.2f ppb\033[0m\n", tvoc);
            }
            // Battery (genPowerCfg)
            else if (af->clusterId == AQARA_TVOC_POWER_CLUSTER && attr == 0x0021 && dataType == 0x20 && zclLen >= 4) {
                uint8_t battRaw = zcl[3];
                LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;32mBattery: %d%%\033[0m\n", battRaw / 2);
            }
        }
    }
}

static void *AqaraTvoc_Thread(void *arg)
{
    (void)arg;
    while (1) {
        SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop(&s_inbox);
        if (!msg) continue;
        if (msg->kind == SENSOR_MSG_AF) HandleAf(&msg->af);
        free(msg);
    }
    return NULL;
}

void AqaraTvoc_Init(void)
{
    memset(g_aqaraTvocs, 0, sizeof(g_aqaraTvocs));
    g_numAqaraTvocs = 0;
    MsgQueue_Init(&s_inbox);
}

void AqaraTvoc_Start(void)
{
    pthread_create(&s_thread, NULL, AqaraTvoc_Thread, NULL);
}

void AqaraTvoc_PostAf(uint16_t addr, const AF_MSG_T *af)
{
    SENSOR_MSG_T *m = calloc(1, sizeof(SENSOR_MSG_T));
    if (m) {
        m->kind = SENSOR_MSG_AF;
        m->shortAddr = addr;
        m->af = *af;
        MsgQueue_Push(&s_inbox, m);
    }
}

void AqaraTvoc_Discover(uint16_t addr, uint8_t ep)
{
    pthread_mutex_lock(&g_deviceMutex);
    int idx = -1;
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == addr) { idx = i; break; }
    }
    if (idx == -1 && g_numAqaraTvocs < MAX_AQARA_TVOC) {
        LOG_EVENT("AQARA_TVOC", addr, "Network Join\n");
        g_aqaraTvocs[g_numAqaraTvocs].shortAddr = addr;
        g_aqaraTvocs[g_numAqaraTvocs].endpoint = ep;
        g_aqaraTvocs[g_numAqaraTvocs].lastSeen = ZNP_GetCurrentTime();
        g_numAqaraTvocs++;
    }
    pthread_mutex_unlock(&g_deviceMutex);
}

bool AqaraTvoc_IsKnown(uint16_t addr)
{
    bool known = false;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == addr) { known = true; break; }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    return known;
}

void AqaraTvoc_UpdateSeen(uint16_t addr)
{
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == addr) {
            g_aqaraTvocs[i].lastSeen = ZNP_GetCurrentTime();
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
}

void AqaraTvoc_ReadEnvironment(uint16_t addr)
{
    pthread_mutex_lock(&g_deviceMutex);
    int idx = -1;
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == addr) { idx = i; break; }
    }
    if (idx != -1) {
        uint8_t ep = g_aqaraTvocs[idx].endpoint;
        pthread_mutex_unlock(&g_deviceMutex);
        
        static uint8_t seq = 0;
        // Read Temp (attr 0x0000)
        uint8_t reqTemp[5] = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, ep, 0, 8, AQARA_TVOC_TEMP_CLUSTER, seq, 0, 30, reqTemp, 5 );
        usleep(20000);
        
        // Read Humidity (attr 0x0000)
        uint8_t reqHum[5] = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, ep, 0, 8, AQARA_TVOC_HUM_CLUSTER, seq, 0, 30, reqHum, 5 );
        usleep(20000);
        
        // Read TVOC (genAnalogInput attr 0x0055)
        uint8_t reqTvoc[5] = { 0x00, ++seq, 0x00, 0x55, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, ep, 0, 8, AQARA_TVOC_ANALOG_CLUSTER, seq, 0, 30, reqTvoc, 5 );
        usleep(20000);
        
        // Read Battery (attr 0x0021)
        uint8_t reqBatt[5] = { 0x00, ++seq, 0x00, 0x21, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, ep, 0, 8, AQARA_TVOC_POWER_CLUSTER, seq, 0, 30, reqBatt, 5 );
    } else {
        pthread_mutex_unlock(&g_deviceMutex);
    }
}

#endif
