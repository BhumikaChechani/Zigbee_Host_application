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
                if (cmdId == 0x0A)
                    LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;36mTemperature: %.2f°C\033[0m\n", tempC);
                
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
                if (cmdId == 0x0A)
                    LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;36mHumidity: %.2f%%\033[0m\n", humPercent);
                
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
                
                const char* quality;
                const char* color;
                TVOC_AQ_STATE_T newState;
                
                if (tvoc <= 65.0f) { quality = "Excellent"; color = "\033[1;32m"; newState = TVOC_AQ_EXCELLENT; }
                else if (tvoc <= 220.0f) { quality = "Good"; color = "\033[1;36m"; newState = TVOC_AQ_GOOD; }
                else if (tvoc <= 660.0f) { quality = "Moderate"; color = "\033[1;33m"; newState = TVOC_AQ_MODERATE; }
                else if (tvoc <= 2200.0f) { quality = "Poor"; color = "\033[1;35m"; newState = TVOC_AQ_POOR; }
                else { quality = "Unhealthy"; color = "\033[1;31m"; newState = TVOC_AQ_UNHEALTHY; }
                
                if (cmdId == 0x0A)
                    LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;35mTVOC: %.2f ppb\033[0m (Air Quality: %s%s\033[0m)\n", tvoc, color, quality);
                
                pthread_mutex_lock(&g_deviceMutex);
                for (int i = 0; i < g_numAqaraTvocs; i++) {
                    if (g_aqaraTvocs[i].shortAddr == af->srcAddr) {
                        if (g_aqaraTvocs[i].lastAirQuality != TVOC_AQ_UNKNOWN && g_aqaraTvocs[i].lastAirQuality != newState) {
                            LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;33m[AIR QUALITY ALERT] Changed to %s%s\033[0m (Reading: %.2f ppb)\n", color, quality, tvoc);
                        }
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
                    if (cmdId == 0x0A)
                        LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;32mBattery: %d%%\033[0m\n", battRaw / 2);
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
                    
                    if (cmdId == 0x0A)
                        LOG_EVENT("AQARA_TVOC", af->srcAddr, "\033[1;32mBattery: %.1fV (~%d%%)\033[0m\n", voltage, pct);
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
        
        double now = ZNP_GetCurrentTime();
        char ts[32];
        
        printf("\n\033[1;36m--- Aqara TVOC Environment Cache (0x%04X) ---\033[0m\n", addr);
        
        if (t.lastTempTime > 0) {
            format_time(now - t.lastTempTime, ts);
            printf("  \033[1;31mTemperature :\033[0m %6.2f C    \033[90m(updated %s)\033[0m\n", t.lastTemp, ts);
        } else {
            printf("  \033[1;31mTemperature :\033[0m \033[90m[Waiting for data...]\033[0m\n");
        }
        
        if (t.lastHumTime > 0) {
            format_time(now - t.lastHumTime, ts);
            printf("  \033[1;34mHumidity    :\033[0m %6.2f %%   \033[90m(updated %s)\033[0m\n", t.lastHum, ts);
        } else {
            printf("  \033[1;34mHumidity    :\033[0m \033[90m[Waiting for data...]\033[0m\n");
        }
        
        if (t.lastTvocTime > 0) {
            format_time(now - t.lastTvocTime, ts);
            printf("  \033[1;35mTVOC        :\033[0m %6.2f ppb \033[90m(updated %s)\033[0m\n", t.lastTvoc, ts);
        } else {
            printf("  \033[1;35mTVOC        :\033[0m \033[90m[Waiting for data...]\033[0m\n");
        }
        
        if (t.lastBattTime > 0) {
            format_time(now - t.lastBattTime, ts);
            printf("  \033[1;32mBattery     :\033[0m %6d %%   \033[90m(updated %s)\033[0m\n", t.lastBatt, ts);
        } else {
            printf("  \033[1;32mBattery     :\033[0m \033[90m[Waiting for data...]\033[0m\n");
        }
        printf("\033[1;36m---------------------------------------------\033[0m\n");
        printf("\033[90m  [Read requests queued. Press sensor button to refresh.]\033[0m\n\n");
        
        // Always queue read requests so the sensor can answer whenever it wakes
        static uint8_t seq = 0;
        uint8_t reqTemp[5] = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, t.endpoint, 0, 8, AQARA_TVOC_TEMP_CLUSTER, seq, 0, 30, reqTemp, 5 );
        usleep(250000);
        
        uint8_t reqHum[5] = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, t.endpoint, 0, 8, AQARA_TVOC_HUM_CLUSTER, seq, 0, 30, reqHum, 5 );
        usleep(250000);
        
        uint8_t reqTvoc[5] = { 0x00, ++seq, 0x00, 0x55, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, t.endpoint, 0, 8, AQARA_TVOC_ANALOG_CLUSTER, seq, 0, 30, reqTvoc, 5 );
        usleep(250000);
        
        uint8_t reqBatt[5] = { 0x00, ++seq, 0x00, 0x20, 0x00 };
        ZNP_AfDataRequestExt( 2, addr, t.endpoint, 0, 8, AQARA_TVOC_POWER_CLUSTER, seq, 0, 30, reqBatt, 5 );
    } else {
        pthread_mutex_unlock(&g_deviceMutex);
    }
}

void AqaraTvoc_PrintStatus(void)
{
    pthread_mutex_lock(&g_deviceMutex);
    printf("Registered Aqara TVOC Sensors (%d):\n", g_numAqaraTvocs);
    double now = ZNP_GetCurrentTime();
    for (int i = 0; i < g_numAqaraTvocs; i++) {
        double diff = now - g_aqaraTvocs[i].lastSeen;
        printf("  - 0x%04X: IEEE=", g_aqaraTvocs[i].shortAddr);
        if (g_aqaraTvocs[i].hasIeee) {
            for (int j = 7; j >= 0; j--) {
                printf("%02x", g_aqaraTvocs[i].ieee[j]);
            }
        } else {
            printf("Unknown");
        }
        printf(", ep=0x%02X, seen=%.1fs ago\n", 
            g_aqaraTvocs[i].endpoint, diff);
    }
    pthread_mutex_unlock(&g_deviceMutex);
}

#endif
