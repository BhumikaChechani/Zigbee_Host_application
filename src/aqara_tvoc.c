#include "aqara_tvoc.h"
#include "config.h"
#include "logger.h"
#include "msg_queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include "znp_host.h"
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

    // DEBUG: print raw payload
    printf("\033[90m[DEBUG] TVOC RAW from 0x%04X clst 0x%04X: \033[0m", af->srcAddr, af->clusterId);
    for (int i=0; i<af->dataLen; i++) {
        printf("\033[90m%02X \033[0m", af->data[i]);
    }
    printf("\n");

    if (cmdId == 0x0A || cmdId == 0x01) { // Report or Read Rsp
        if (zclLen >= 4) {
            uint16_t attr = zcl[0] | (zcl[1] << 8);
            
            bool success = (cmdId == 0x0A) || (cmdId == 0x01 && zcl[2] == 0x00);
            if (!success) {
                if (cmdId == 0x01 && zcl[2] != 0x00) {
                    LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;31mZCL Error (cluster 0x%04X, attr 0x%04X): status 0x%02X\033[0m\n", af->clusterId, attr, zcl[2]);
                }
                return;
            }

            uint8_t typeOffset = (cmdId == 0x01) ? 3 : 2;
            uint8_t dataOffset = (cmdId == 0x01) ? 4 : 3;

            if (zclLen < dataOffset + 1) return;
            
            uint8_t dataType = zcl[typeOffset];
            const uint8_t *data = &zcl[dataOffset];
            
            // Temperature
            if (af->clusterId == AQARA_TVOC_TEMP_CLUSTER && attr == 0x0000 && dataType == 0x29 && zclLen >= dataOffset + 2) {
                int16_t tempRaw = data[0] | (data[1] << 8);
                float tempC = tempRaw / 100.0f;
                
                pthread_mutex_lock(&g_deviceMutex);
                for (int i = 0; i < g_numAqaraTvocs; i++) {
                    if (g_aqaraTvocs[i].shortAddr == af->srcAddr) {
                        g_aqaraTvocs[i].lastTemp = tempC;
                        g_aqaraTvocs[i].lastTempTime = ZNP_GetCurrentTime();
                        break;
                    }
                }
                pthread_mutex_unlock(&g_deviceMutex);
            }
            // Humidity
            else if (af->clusterId == AQARA_TVOC_HUM_CLUSTER && attr == 0x0000 && dataType == 0x21 && zclLen >= dataOffset + 2) {
                uint16_t humRaw = data[0] | (data[1] << 8);
                float humPercent = humRaw / 100.0f;
                
                pthread_mutex_lock(&g_deviceMutex);
                for (int i = 0; i < g_numAqaraTvocs; i++) {
                    if (g_aqaraTvocs[i].shortAddr == af->srcAddr) {
                        g_aqaraTvocs[i].lastHum = humPercent;
                        g_aqaraTvocs[i].lastHumTime = ZNP_GetCurrentTime();
                        break;
                    }
                }
                pthread_mutex_unlock(&g_deviceMutex);
            }
            // TVOC (genAnalogInput, presentValue is attr 0x0055, type single precision float 0x39)
            else if (af->clusterId == AQARA_TVOC_ANALOG_CLUSTER && attr == 0x0055 && dataType == 0x39 && zclLen >= dataOffset + 4) {
                float tvoc = 0.0f;
                memcpy(&tvoc, &data[0], 4);
                
                TVOC_AQ_STATE_T newState;
                
                if (tvoc <= 65.0f) { newState = TVOC_AQ_EXCELLENT; }
                else if (tvoc <= 220.0f) { newState = TVOC_AQ_GOOD; }
                else if (tvoc <= 660.0f) { newState = TVOC_AQ_MODERATE; }
                else if (tvoc <= 2200.0f) { newState = TVOC_AQ_POOR; }
                else { newState = TVOC_AQ_UNHEALTHY; }
                
                pthread_mutex_lock(&g_deviceMutex);
                for (int i = 0; i < g_numAqaraTvocs; i++) {
                    if (g_aqaraTvocs[i].shortAddr == af->srcAddr) {
                        g_aqaraTvocs[i].lastAirQuality = newState;
                        g_aqaraTvocs[i].lastTvoc = tvoc;
                        g_aqaraTvocs[i].lastTvocTime = ZNP_GetCurrentTime();
                        break;
                    }
                }
                pthread_mutex_unlock(&g_deviceMutex);
            }
            // Battery (genPowerCfg)
            else if (af->clusterId == AQARA_TVOC_POWER_CLUSTER && dataType == 0x20 && zclLen >= dataOffset + 1) {
                uint8_t battRaw = data[0];
                
                if (attr == 0x0021) {
                    pthread_mutex_lock(&g_deviceMutex);
                    for (int i = 0; i < g_numAqaraTvocs; i++) {
                        if (g_aqaraTvocs[i].shortAddr == af->srcAddr) {
                            g_aqaraTvocs[i].lastBatt = battRaw / 2;
                            g_aqaraTvocs[i].lastBattTime = ZNP_GetCurrentTime();
                            break;
                        }
                    }
                    pthread_mutex_unlock(&g_deviceMutex);
                } else if (attr == 0x0020) { // Battery Voltage (units of 100mV)
                    float voltage = battRaw / 10.0f;
                    int pct = (int)(((voltage - 2.5f) / 0.5f) * 100);
                    if (pct > 100) pct = 100;
                    if (pct < 0) pct = 0;
                    
                    pthread_mutex_lock(&g_deviceMutex);
                    for (int i = 0; i < g_numAqaraTvocs; i++) {
                        if (g_aqaraTvocs[i].shortAddr == af->srcAddr) {
                            g_aqaraTvocs[i].lastBatt = pct;
                            g_aqaraTvocs[i].lastBattTime = ZNP_GetCurrentTime();
                            break;
                        }
                    }
                    pthread_mutex_unlock(&g_deviceMutex);
                }
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
        if (msg->kind == SENSOR_MSG_ASSIGN) {
            AqaraTvoc_Setup(msg->shortAddr);
        } else if (msg->kind == SENSOR_MSG_AF) {
            HandleAf(&msg->af);
        }
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

void AqaraTvoc_PostAssign(uint16_t shortAddr_)
{
    SENSOR_MSG_T *m = calloc(1, sizeof(SENSOR_MSG_T));
    if (m) {
        m->kind = SENSOR_MSG_ASSIGN;
        m->shortAddr = shortAddr_;
        MsgQueue_Push(&s_inbox, m);
    }
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
    uint8_t ieee[8];
    bool hasIeee = Device_GetDiscoveredIeee(addr, ieee);
    bool changed = false;

    if (hasIeee) {
        for (int i = 0; i < g_numAqaraTvocs; i++) {
            if (g_aqaraTvocs[i].hasIeee && memcmp(g_aqaraTvocs[i].ieee, ieee, 8) == 0) {
                idx = i;
                if (g_aqaraTvocs[i].shortAddr != addr) {
                    LOG_EVENT("AQARA_TVOC", addr, "Network Rejoin (Short address changed from 0x%04X)\n", g_aqaraTvocs[i].shortAddr);
                    g_aqaraTvocs[i].shortAddr = addr;
                    g_aqaraTvocs[i].configured = false;
                    changed = true;
                }
                break;
            }
        }
    }

    if (idx == -1) {
        for (int i = 0; i < g_numAqaraTvocs; i++) {
            if (g_aqaraTvocs[i].shortAddr == addr) { idx = i; break; }
        }
    }

    if (idx == -1 && g_numAqaraTvocs < MAX_AQARA_TVOC) {
        LOG_EVENT("AQARA_TVOC", addr, "Network Join\n");
        g_aqaraTvocs[g_numAqaraTvocs].shortAddr = addr;
        g_aqaraTvocs[g_numAqaraTvocs].endpoint = ep;
        g_aqaraTvocs[g_numAqaraTvocs].lastSeen = ZNP_GetCurrentTime();
        if (hasIeee) {
            g_aqaraTvocs[g_numAqaraTvocs].hasIeee = true;
            memcpy(g_aqaraTvocs[g_numAqaraTvocs].ieee, ieee, 8);
        }
        g_aqaraTvocs[g_numAqaraTvocs].configured = false;
        g_aqaraTvocs[g_numAqaraTvocs].setupRetries = 0;
        g_aqaraTvocs[g_numAqaraTvocs].lastSetupAttempt = 0.0;
        g_numAqaraTvocs++;
        changed = true;
    } else if (idx != -1) {
        if (g_aqaraTvocs[idx].endpoint != ep) {
            g_aqaraTvocs[idx].endpoint = ep;
            changed = true;
        }
        g_aqaraTvocs[idx].lastSeen = ZNP_GetCurrentTime();
        if (hasIeee && !g_aqaraTvocs[idx].hasIeee) {
            g_aqaraTvocs[idx].hasIeee = true;
            memcpy(g_aqaraTvocs[idx].ieee, ieee, 8);
            changed = true;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    
    if (changed) {
        Device_Save();
    }
    
    AqaraTvoc_PostAssign(addr);
}

void AqaraTvoc_Setup(uint16_t shortAddr_)
{
    pthread_mutex_lock(&g_deviceMutex);
    int idx = -1;
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == shortAddr_) {
            idx = i;
            break;
        }
    }
    
    if (idx == -1 || g_aqaraTvocs[idx].configured) {
        pthread_mutex_unlock(&g_deviceMutex);
        return;
    }
    
    if (!g_aqaraTvocs[idx].hasIeee) {
        g_aqaraTvocs[idx].hasIeee = Device_GetDiscoveredIeee(shortAddr_, g_aqaraTvocs[idx].ieee);
    }
    if (!g_aqaraTvocs[idx].hasIeee) {
        pthread_mutex_unlock(&g_deviceMutex);
        LOG_DEBUG("Aqara TVOC 0x%04X missing IEEE - requesting...\n", shortAddr_);
        uint8_t reqPay[4] = { shortAddr_ & 0xFF, (shortAddr_ >> 8) & 0xFF, 0x01, 0x00 };
        ZNP_Sreq(0x25, 0x01, reqPay, 4, NULL, 3000);
        return;
    }

    uint8_t sensorIeee[8];
    uint8_t endpoint = g_aqaraTvocs[idx].endpoint;
    memcpy(sensorIeee, g_aqaraTvocs[idx].ieee, 8);
    g_aqaraTvocs[idx].lastSetupAttempt = ZNP_GetCurrentTime();
    pthread_mutex_unlock(&g_deviceMutex);

    LOG_DEBUG("Configuring Aqara TVOC 0x%04X...\n", shortAddr_);

    // Bind Temp, Humidity, and TVOC Analog clusters
    ZNP_ZdoBindReq(shortAddr_, sensorIeee, endpoint, AQARA_TVOC_TEMP_CLUSTER, g_coordinatorIeee, 8);
    usleep(300000);
    ZNP_ZdoBindReq(shortAddr_, sensorIeee, endpoint, AQARA_TVOC_HUM_CLUSTER, g_coordinatorIeee, 8);
    usleep(300000);
    ZNP_ZdoBindReq(shortAddr_, sensorIeee, endpoint, AQARA_TVOC_ANALOG_CLUSTER, g_coordinatorIeee, 8);
    usleep(300000);

    pthread_mutex_lock(&g_deviceMutex);
    if (idx < g_numAqaraTvocs && g_aqaraTvocs[idx].shortAddr == shortAddr_) {
        g_aqaraTvocs[idx].configured = true;
    }
    pthread_mutex_unlock(&g_deviceMutex);
    
    LOG_DEBUG("Aqara TVOC 0x%04X configured OK.\n", shortAddr_);
    Device_Save();
}

void AqaraTvoc_UpdateIeee(uint16_t shortAddr_, const uint8_t *ieee_) {
    bool found = false;
    pthread_mutex_lock(&g_deviceMutex);
    int targetIdx = -1;
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == shortAddr_) {
            memcpy(g_aqaraTvocs[i].ieee, ieee_, 8);
            g_aqaraTvocs[i].hasIeee = true;
            targetIdx = i;
            found = true;
            break;
        }
    }
    if (found) {
        for (int i = g_numAqaraTvocs - 1; i >= 0; i--) {
            if (g_aqaraTvocs[i].shortAddr != shortAddr_ && g_aqaraTvocs[i].hasIeee && memcmp(g_aqaraTvocs[i].ieee, ieee_, 8) == 0) {
                // Duplicate resolution
                for (int j = i; j < g_numAqaraTvocs - 1; j++) {
                    g_aqaraTvocs[j] = g_aqaraTvocs[j + 1];
                }
                g_numAqaraTvocs--;
                if (targetIdx > i) targetIdx--;
            }
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        AqaraTvoc_PostAssign(shortAddr_);
    }
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

static void format_time(double diff, char* out) {
    if (diff < 60) sprintf(out, "%.1fs ago", diff);
    else if (diff < 3600) sprintf(out, "%dm %ds ago", (int)(diff/60), (int)diff%60);
    else sprintf(out, "%dh %dm ago", (int)(diff/3600), ((int)diff%3600)/60);
}

void AqaraTvoc_ReadEnvironment(uint16_t addr)
{
    pthread_mutex_lock(&g_deviceMutex);
    int idx = -1;
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        if (g_aqaraTvocs[i].shortAddr == addr) { idx = i; break; }
    }
    if (idx != -1) {
        AQARA_TVOC_T t = g_aqaraTvocs[idx];
        pthread_mutex_unlock(&g_deviceMutex);
        
        printf("\033[90mFetching latest environment data from 0x%04X (takes up to 3s)...\033[0m\n", addr);
        
        double oldTempTime = t.lastTempTime;
        double oldHumTime  = t.lastHumTime;
        double oldTvocTime = t.lastTvocTime;
        
        static uint8_t seq = 0;
        uint8_t reqTemp[5] = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt(2, addr, t.endpoint, 0, 8, AQARA_TVOC_TEMP_CLUSTER, seq, 0, 30, reqTemp, 5);
        
        uint8_t reqHum[5]  = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt(2, addr, t.endpoint, 0, 8, AQARA_TVOC_HUM_CLUSTER, seq, 0, 30, reqHum, 5);
        
        uint8_t reqTvoc[5] = { 0x00, ++seq, 0x00, 0x55, 0x00 };
        ZNP_AfDataRequestExt(2, addr, t.endpoint, 0, 8, AQARA_TVOC_ANALOG_CLUSTER, seq, 0, 30, reqTvoc, 5);
        
        // Wait up to 3 seconds for the sensor to reply
        bool gotNewData = false;
        for (int w = 0; w < 30; w++) {
            usleep(100000); // 100ms
            pthread_mutex_lock(&g_deviceMutex);
            if (g_aqaraTvocs[idx].lastTempTime > oldTempTime || 
                g_aqaraTvocs[idx].lastHumTime > oldHumTime || 
                g_aqaraTvocs[idx].lastTvocTime > oldTvocTime) {
                t = g_aqaraTvocs[idx];
                gotNewData = true;
                pthread_mutex_unlock(&g_deviceMutex);
                break;
            }
            pthread_mutex_unlock(&g_deviceMutex);
        }
        
        // Re-fetch in case it updated slightly
        pthread_mutex_lock(&g_deviceMutex);
        t = g_aqaraTvocs[idx];
        pthread_mutex_unlock(&g_deviceMutex);
        
        double now = ZNP_GetCurrentTime();
        char ts[32];
        
        printf("\n\033[1;36m--- Aqara TVOC Environment Status (0x%04X) ---\033[0m\n", addr);
        
        if (t.lastTempTime > 0) {
            format_time(now - t.lastTempTime, ts);
            printf("  \033[1;31mTemperature :\033[0m %6.2f C    \033[90m(live %s)\033[0m\n", t.lastTemp, ts);
        } else {
            printf("  \033[1;31mTemperature :\033[0m \033[90m[No data]\033[0m\n");
        }
        
        if (t.lastHumTime > 0) {
            format_time(now - t.lastHumTime, ts);
            printf("  \033[1;34mHumidity    :\033[0m %6.2f %%   \033[90m(live %s)\033[0m\n", t.lastHum, ts);
        } else {
            printf("  \033[1;34mHumidity    :\033[0m \033[90m[No data]\033[0m\n");
        }
        
        if (t.lastTvocTime > 0) {
            format_time(now - t.lastTvocTime, ts);
            const char* quality;
            const char* color;
            if (t.lastTvoc <= 65.0f) { quality = "Excellent"; color = "\033[1;32m"; }
            else if (t.lastTvoc <= 220.0f) { quality = "Good"; color = "\033[1;36m"; }
            else if (t.lastTvoc <= 660.0f) { quality = "Moderate"; color = "\033[1;33m"; }
            else if (t.lastTvoc <= 2200.0f) { quality = "Poor"; color = "\033[1;35m"; }
            else { quality = "Unhealthy"; color = "\033[1;31m"; }
            
            printf("  \033[1;35mTVOC        :\033[0m %6.2f ppb %s[%s]\033[0m \033[90m(live %s)\033[0m\n", t.lastTvoc, color, quality, ts);
        } else {
            printf("  \033[1;35mTVOC        :\033[0m \033[90m[No data]\033[0m\n");
        }
        
        if (t.lastBattTime > 0) {
            format_time(now - t.lastBattTime, ts);
            printf("  \033[1;32mBattery     :\033[0m %6d %%   \033[90m(live %s)\033[0m\n", t.lastBatt, ts);
        }
        printf("\033[1;36m----------------------------------------------\033[0m\n");
        
        if (!gotNewData) {
            printf("\033[1;33m[!] Sensor is asleep and did not respond to the query.\033[0m\n");
            printf("\033[90m    Data above is the last known state. Press the button on the sensor to wake it.\033[0m\n\n");
        } else {
            printf("\033[1;32m[✓] Successfully fetched latest real-time data from sensor.\033[0m\n\n");
        }
    } else {
        pthread_mutex_unlock(&g_deviceMutex);
    }
}

void AqaraTvoc_PrintStatus(void)
{
    pthread_mutex_lock(&g_deviceMutex);
    if (g_numAqaraTvocs > 0) {
        printf("\n\033[1;37mRegistered Aqara TVOC Sensors (%d):\033[0m\n", g_numAqaraTvocs);
        double now = ZNP_GetCurrentTime();
        for (int i = 0; i < g_numAqaraTvocs; i++) {
            double diff = now - g_aqaraTvocs[i].lastSeen;
            printf("  - \033[1m0x%04X\033[0m: IEEE=", g_aqaraTvocs[i].shortAddr);
            if (g_aqaraTvocs[i].hasIeee) {
                for (int j = 7; j >= 0; j--) {
                    printf("%02x", g_aqaraTvocs[i].ieee[j]);
                }
            } else {
                printf("Unknown");
            }
            printf(", ep=0x%02X, \033[90mlast_seen=%.1fs ago\033[0m\n", 
                g_aqaraTvocs[i].endpoint, diff);
                
            char ts[32];
            AQARA_TVOC_T t = g_aqaraTvocs[i];
            
            if (t.lastTempTime > 0) {
                format_time(now - t.lastTempTime, ts);
                printf("    \033[1;31mTemperature :\033[0m %6.2f C    \033[90m(live %s)\033[0m\n", t.lastTemp, ts);
            }
            if (t.lastHumTime > 0) {
                format_time(now - t.lastHumTime, ts);
                printf("    \033[1;34mHumidity    :\033[0m %6.2f %%   \033[90m(live %s)\033[0m\n", t.lastHum, ts);
            }
            if (t.lastTvocTime > 0) {
                format_time(now - t.lastTvocTime, ts);
                const char* quality;
                const char* color;
                if (t.lastTvoc <= 65.0f) { quality = "Excellent"; color = "\033[1;32m"; }
                else if (t.lastTvoc <= 220.0f) { quality = "Good"; color = "\033[1;36m"; }
                else if (t.lastTvoc <= 660.0f) { quality = "Moderate"; color = "\033[1;33m"; }
                else if (t.lastTvoc <= 2200.0f) { quality = "Poor"; color = "\033[1;35m"; }
                else { quality = "Unhealthy"; color = "\033[1;31m"; }
                printf("    \033[1;35mTVOC        :\033[0m %6.2f ppb %s[%s]\033[0m \033[90m(live %s)\033[0m\n", t.lastTvoc, color, quality, ts);
            }
            if (t.lastBattTime > 0) {
                format_time(now - t.lastBattTime, ts);
                printf("    \033[1;32mBattery     :\033[0m %6d %%   \033[90m(live %s)\033[0m\n", t.lastBatt, ts);
            }
        }
    } else {
        printf("\n\033[1;37mNo Aqara TVOC Sensors registered.\033[0m\n");
    }
    pthread_mutex_unlock(&g_deviceMutex);
}

#endif
