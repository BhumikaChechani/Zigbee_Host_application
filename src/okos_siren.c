#include "config.h"
#if ENABLE_OKOS_SIREN
/// @file okos_siren.c
/// @brief Okos Smart Siren module implementation.
#include "okos_siren.h"
#include "msg_queue.h"
#include "usecase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

OKOS_SIREN_T g_okosSirens[MAX_OKOS_SIRENS];
int g_numOkosSirens = 0;

static MSG_QUEUE_T s_inbox;
static pthread_t   s_thread;
static uint8_t     s_seq = 0;

static uint8_t NextSeq(void) { return __atomic_add_fetch(&s_seq, 1, __ATOMIC_RELAXED); }

static void SendTuyaDP(uint16_t addr, uint8_t ep, uint8_t dpId, uint8_t dpType, const uint8_t *val, uint16_t valLen);

static void SendTuyaQuery(uint16_t addr, uint8_t ep)
{
    uint8_t payload[5];
    uint8_t seq = NextSeq();
    payload[0] = 0x11; // Cluster-specific, client to server
    payload[1] = seq;  // ZCL TransSeq
    payload[2] = 0x02; // Tuya Cmd 0x02: Get Data / Poll
    payload[3] = 0x00; // TransSeq High
    payload[4] = seq;  // TransSeq Low
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_TUYA_CLUSTER, seq, 0, 30, payload, 5);

    uint8_t seq2 = NextSeq();
    payload[1] = seq2;
    payload[2] = 0x01; // Tuya Cmd 0x01: Report Query
    payload[4] = seq2;
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_TUYA_CLUSTER, seq2, 0, 30, payload, 5);

    uint8_t seq3 = NextSeq();
    payload[1] = seq3;
    payload[2] = 0x03; // Tuya Cmd 0x03: DP Query (used in some firmwares)
    payload[4] = seq3;
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_TUYA_CLUSTER, seq3, 0, 30, payload, 5);

    // Query standard attributes and Tuya's hidden battery attributes on Basic Cluster
    uint8_t seq4 = NextSeq();
    // 0x0001=AppVersion, 0x0004=Manufacturer, 0xFFE2=TuyaMCU, 0xFFE4=TuyaPower, 0xFFDF=TuyaState
    uint8_t zclBasic[15] = { 0x00, seq4, 0x00, 
                             0x01, 0x00, 0x04, 0x00, 
                             0xE2, 0xFF, 0xE4, 0xFF, 0xDF, 0xFF };
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, 0x0000, seq4, 0, 30, zclBasic, 13);
}



static void HandleAf(const AF_MSG_T *af)
{
    if (af->dataLen < 3) return;
    uint8_t fc = af->data[0];
    int hdrLen = (fc & 0x04) ? 5 : 3;
    if (af->dataLen < hdrLen) return;
    uint8_t cmdId = af->data[hdrLen-1];
    const uint8_t *zcl = &af->data[hdrLen];
    int zclLen = af->dataLen - hdrLen;

    if (af->clusterId == OKOS_IAS_ZONE_CLUSTER) {
        if (cmdId == 0x01 && zclLen >= 2) { // Zone Enroll Request
            uint16_t zoneType = zcl[0] | (zcl[1] << 8);
            OkosSiren_HandleEnroll(af->srcAddr, af->srcEp, af->data[hdrLen-2], zoneType);
        } else if (cmdId == 0x00 && zclLen >= 2) { // Zone Status Change
            uint16_t zoneStatus = zcl[0] | (zcl[1] << 8);
            bool tamper = (zoneStatus & 0x0004) != 0;
            ZNP_SendDefaultResponse(af->srcAddr, af->srcEp, OKOS_IAS_ZONE_CLUSTER, af->data[hdrLen-2], 0x00, 0x00);
            pthread_mutex_lock(&g_deviceMutex);
            bool changed = false;
            for (int i = 0; i < g_numOkosSirens; i++) {
                if (g_okosSirens[i].shortAddr == af->srcAddr) {
                    if (g_okosSirens[i].isTampered != tamper) {
                        g_okosSirens[i].isTampered = tamper;
                        changed = true;
                    }
                    break;
                }
            }
            pthread_mutex_unlock(&g_deviceMutex);
            if (changed)
                UseCase_Post(tamper ? UC_TAMPER_DETECTED : UC_TAMPER_CLEARED, af->srcAddr, zoneStatus, 0);
        }
    } else if (af->clusterId == OKOS_POWER_CLUSTER) {
        if (cmdId == 0x01 && zclLen >= 5 && zcl[0]==0x21 && zcl[1]==0x00 && zcl[2]==0x00) {
            uint8_t pct = zcl[4];
            LOG_EVENT("OKOS_SIREN", af->srcAddr, "Battery: %u%%\n", pct);
            pthread_mutex_lock(&g_deviceMutex);
            for (int i = 0; i < g_numOkosSirens; i++) {
                if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].batteryPct = pct; break; }
            }
            pthread_mutex_unlock(&g_deviceMutex);
        }
    } else if (af->clusterId == OKOS_TEMP_CLUSTER) {
        if (cmdId == 0x0A && zclLen >= 5 && zcl[0]==0x00 && zcl[1]==0x00) {
            int16_t temp = (int16_t)(zcl[3] | (zcl[4] << 8));
            LOG_EVENT("OKOS_SIREN", af->srcAddr, "Temperature: %.2f C\n", temp/100.0f);
            pthread_mutex_lock(&g_deviceMutex);
            for (int i = 0; i < g_numOkosSirens; i++) {
                if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].temperatureCdeg = temp; break; }
            }
            pthread_mutex_unlock(&g_deviceMutex);
        }
    } else if (af->clusterId == OKOS_HUM_CLUSTER) {
        if (cmdId == 0x0A && zclLen >= 5 && zcl[0]==0x00 && zcl[1]==0x00) {
            uint16_t hum = zcl[3] | (zcl[4] << 8);
            LOG_EVENT("OKOS_SIREN", af->srcAddr, "Humidity: %.2f%%\n", hum/100.0f);
            pthread_mutex_lock(&g_deviceMutex);
            for (int i = 0; i < g_numOkosSirens; i++) {
                if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].humidityHpct = hum; break; }
            }
            pthread_mutex_unlock(&g_deviceMutex);
        }
    } else if (af->clusterId == OKOS_TUYA_CLUSTER) {
        LOG_DEBUG("OKOS_SIREN 0x%04X: Tuya AF payload len=%d\n", af->srcAddr, af->dataLen);

        if (af->dataLen < 3) return;
        uint8_t tuyaCmd   = af->data[2];
        uint8_t tuyaSeqHi = (af->dataLen >= 4) ? af->data[3] : 0;
        uint8_t tuyaSeqLo = (af->dataLen >= 5) ? af->data[4] : 0;

        // --- Tuya Time Sync (Cmd 0x24): device requests current time ---
        if (tuyaCmd == 0x24) {
            time_t now = time(NULL);
            uint32_t ts = (uint32_t)now;
            uint8_t rsp[13];
            uint8_t rseq = NextSeq();
            rsp[0] = 0x11;
            rsp[1] = rseq;
            rsp[2] = 0x24; // Time sync response
            rsp[3] = tuyaSeqHi;
            rsp[4] = tuyaSeqLo;
            // Standard time (UTC seconds since 2000-01-01)
            uint32_t tuya_epoch = (ts > 946684800u) ? (ts - 946684800u) : 0;
            rsp[5] = (tuya_epoch >> 24) & 0xFF;
            rsp[6] = (tuya_epoch >> 16) & 0xFF;
            rsp[7] = (tuya_epoch >>  8) & 0xFF;
            rsp[8] = (tuya_epoch      ) & 0xFF;
            // Local time offset (same as UTC for simplicity)
            rsp[9] = rsp[5]; rsp[10] = rsp[6]; rsp[11] = rsp[7]; rsp[12] = rsp[8];
            ZNP_AfDataRequestExt(2, af->srcAddr, af->srcEp, 0, 8, OKOS_TUYA_CLUSTER, rseq, 0, 30, rsp, 13);
            LOG_DEBUG("[OKOS] Replied to Tuya Time Sync from 0x%04X\n", af->srcAddr);
        }

        // --- Tuya Status Report / Join Report (Cmd 0x01 or 0x02): send ACK ---
        if (tuyaCmd == 0x01 || tuyaCmd == 0x02) {
            // ACK: echo back Cmd 0x01 with same Tuya sequence, no DP payload
            uint8_t ack[5];
            uint8_t aseq = NextSeq();
            ack[0] = 0x11;
            ack[1] = aseq;
            ack[2] = 0x01; // Tuya Query/ACK
            ack[3] = tuyaSeqHi;
            ack[4] = tuyaSeqLo;
            ZNP_AfDataRequestExt(2, af->srcAddr, af->srcEp, 0, 8, OKOS_TUYA_CLUSTER, aseq, 0, 30, ack, 5);
            LOG_DEBUG("[OKOS] ACK'd Tuya Cmd 0x%02X from 0x%04X\n", tuyaCmd, af->srcAddr);
        }

        if (af->dataLen >= 9) {
            if (tuyaCmd == 0x01 || tuyaCmd == 0x02 || tuyaCmd == 0x05 || tuyaCmd == 0x06) {
                int offset = 5;
                while (offset + 4 <= af->dataLen) {
                    uint8_t dpId   = af->data[offset];
                    uint8_t dpType = af->data[offset + 1];
                    uint16_t dpLen = (af->data[offset + 2] << 8) | af->data[offset + 3];
                    if (offset + 4 + dpLen > af->dataLen) break;
                    const uint8_t *val = &af->data[offset + 4];

                    uint32_t numVal = 0;
                    if (dpType == 0x01 || dpType == 0x04) {
                        numVal = val[0];
                    } else if (dpType == 0x02 && dpLen == 4) {
                        numVal = (val[0]<<24) | (val[1]<<16) | (val[2]<<8) | val[3];
                    } else if (dpType == 0x02 && dpLen == 2) {
                        numVal = (val[0]<<8) | val[1];
                    } else if (dpType == 0x02 && dpLen == 1) {
                        numVal = val[0];
                    }

                    if (dpId == 15 || dpId == 101 || dpId == 114 || dpId == 115) { // Battery %
                        uint8_t bat = (numVal > 100) ? 100 : (uint8_t)numVal;
                        LOG_EVENT("OKOS_SIREN", af->srcAddr, "Battery: %u%%\n", bat);
                        pthread_mutex_lock(&g_deviceMutex);
                        for (int i = 0; i < g_numOkosSirens; i++)
                            if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].batteryPct = bat; break; }
                        pthread_mutex_unlock(&g_deviceMutex);
                    } else if (dpId == 106 || dpId == 108 || dpId == 109) { // Temp
                        float temp = (numVal > 1000) ? (numVal / 100.0f) : (numVal / 10.0f);
                        int16_t tempCdeg = (int16_t)(temp * 100);
                        LOG_EVENT("OKOS_SIREN", af->srcAddr, "Temperature: %.2f C\n", temp);
                        pthread_mutex_lock(&g_deviceMutex);
                        for (int i = 0; i < g_numOkosSirens; i++)
                            if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].temperatureCdeg = tempCdeg; break; }
                        pthread_mutex_unlock(&g_deviceMutex);
                    } else if (dpId == 107 || dpId == 110) { // Humidity
                        float hum = (numVal > 100) ? (numVal / 10.0f) : (float)numVal;
                        uint16_t humHpct = (uint16_t)(hum * 100);
                        LOG_EVENT("OKOS_SIREN", af->srcAddr, "Humidity: %.2f%%\n", hum);
                        pthread_mutex_lock(&g_deviceMutex);
                        for (int i = 0; i < g_numOkosSirens; i++)
                            if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].humidityHpct = humHpct; break; }
                        pthread_mutex_unlock(&g_deviceMutex);
                    } else if (dpId == 104 || dpId == 21) { // Tone
                        uint8_t t = (uint8_t)numVal + 1;
                        pthread_mutex_lock(&g_deviceMutex);
                        for (int i = 0; i < g_numOkosSirens; i++)
                            if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].toneId = t; break; }
                        pthread_mutex_unlock(&g_deviceMutex);
                        LOG_EVENT("OKOS_SIREN", af->srcAddr, "Reported Tone: %u\n", t);
                    } else if (dpId == 116 || dpId == 5) { // Volume
                        uint8_t v = (uint8_t)numVal;
                        pthread_mutex_lock(&g_deviceMutex);
                        for (int i = 0; i < g_numOkosSirens; i++)
                            if (g_okosSirens[i].shortAddr == af->srcAddr) { g_okosSirens[i].volume = v; break; }
                        pthread_mutex_unlock(&g_deviceMutex);
                        LOG_EVENT("OKOS_SIREN", af->srcAddr, "Reported Volume: %u\n", v);
                    } else {
                        LOG_DEBUG("[OKOS_SIREN 0x%04X] Unhandled Tuya DP ID=%u Type=%u Len=%u Val=%u\n",
                               af->srcAddr, dpId, dpType, dpLen, numVal);
                    }
                    offset += 4 + dpLen;
                }
            }
        }
    } else if (af->clusterId == 0x0000) {
        // Tuya devices often return MCU version or custom data on the Basic cluster
        LOG_DEBUG("OKOS_SIREN 0x%04X: Basic Cluster (0x0000) payload len=%d\n", af->srcAddr, af->dataLen);
        
        // Check for Tuya specific attribute 0xFFE2 / 0xFFE4 which might contain battery
        if (af->dataLen >= 6 && af->data[0] == 0x08) {
            uint16_t attrId = (af->data[3] << 8) | af->data[2];
            LOG_DEBUG("Parsed Basic Attribute Report: Attr=0x%04X\n", attrId);
        }
    } else {
        // Other clusters (e.g. ZCL Power 0x0001, Temp 0x0402)
        if (af->clusterId == OKOS_POWER_CLUSTER || af->clusterId == OKOS_TEMP_CLUSTER || af->clusterId == OKOS_HUM_CLUSTER) {
            LOG_DEBUG("OKOS_SIREN 0x%04X: Cluster 0x%04X payload len=%d\n", af->srcAddr, af->clusterId, af->dataLen);
        }
    }
}

static void *OkosSiren_Thread(void *arg)
{
    (void)arg;
    while (1) {
        SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop(&s_inbox);
        if (!msg) continue;
        if      (msg->kind == SENSOR_MSG_ASSIGN) OkosSiren_Setup(msg->shortAddr);
        else if (msg->kind == SENSOR_MSG_AF)      HandleAf(&msg->af);
        free(msg);
    }
    return NULL;
}

// --------------- API ---------------
void OkosSiren_Init(void)
{
    memset(g_okosSirens, 0, sizeof(g_okosSirens));
    g_numOkosSirens = 0;
    MsgQueue_Init(&s_inbox);
}

void OkosSiren_Start(void) { pthread_create(&s_thread, NULL, OkosSiren_Thread, NULL); }

static SENSOR_MSG_T *MkMsg(SENSOR_MSG_KIND_T kind, uint16_t addr)
{
    SENSOR_MSG_T *m = calloc(1, sizeof(SENSOR_MSG_T));
    if (m) { m->kind = kind; m->shortAddr = addr; }
    return m;
}

void OkosSiren_PostAssign(uint16_t a) { SENSOR_MSG_T *m = MkMsg(SENSOR_MSG_ASSIGN, a); if (m) MsgQueue_Push(&s_inbox, m); }


void OkosSiren_PostAf(uint16_t addr, const AF_MSG_T *af)
{
    SENSOR_MSG_T *m = MkMsg(SENSOR_MSG_AF, addr);
    if (m) { m->af = *af; MsgQueue_Push(&s_inbox, m); }
}

void OkosSiren_Discover(uint16_t addr, uint8_t ep)
{
    pthread_mutex_lock(&g_deviceMutex);
    int idx = -1;
    for (int i = 0; i < g_numOkosSirens; i++) { if (g_okosSirens[i].shortAddr == addr) { idx = i; break; } }
    bool changed = false;
    if (idx == -1 && g_numOkosSirens < MAX_OKOS_SIRENS) {
        LOG_EVENT("OKOS_SIREN", addr, "Network Join\n");
        OKOS_SIREN_T *s = &g_okosSirens[g_numOkosSirens++];
        s->shortAddr = addr; s->endpoint = ep;
        s->lastSeen = ZNP_GetCurrentTime();
        s->hasIeee = Device_GetDiscoveredIeee(addr, s->ieee);
        s->volume = 2; s->toneId = OKOS_TONE_BURGLAR; s->strobeMode = OKOS_STROBE_NONE;
        s->temperatureCdeg = 0x7FFF; s->humidityHpct = 0xFFFF; s->batteryPct = 0xFF;
        changed = true;
    } else if (idx >= 0) {
        g_okosSirens[idx].lastSeen = ZNP_GetCurrentTime();
        if (g_okosSirens[idx].endpoint != ep) { g_okosSirens[idx].endpoint = ep; changed = true; }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (changed) Device_Save();
    OkosSiren_PostAssign(addr);
}

void OkosSiren_UpdateIeee(uint16_t addr, const uint8_t *ieee)
{
    bool found = false;
    pthread_mutex_lock(&g_deviceMutex);
    int tgt = -1;
    for (int i = 0; i < g_numOkosSirens; i++) {
        if (g_okosSirens[i].shortAddr == addr) {
            memcpy(g_okosSirens[i].ieee, ieee, 8); g_okosSirens[i].hasIeee = true;
            tgt = i; found = true; break;
        }
    }
    if (found) {
        for (int i = g_numOkosSirens-1; i >= 0; i--) {
            if (g_okosSirens[i].shortAddr != addr && g_okosSirens[i].hasIeee &&
                memcmp(g_okosSirens[i].ieee, ieee, 8) == 0) {
                g_okosSirens[tgt].zoneId    = g_okosSirens[i].zoneId;
                g_okosSirens[tgt].configured= g_okosSirens[i].configured;
                g_okosSirens[tgt].volume    = g_okosSirens[i].volume;
                g_okosSirens[tgt].toneId    = g_okosSirens[i].toneId;
                for (int j = i; j < g_numOkosSirens-1; j++) g_okosSirens[j] = g_okosSirens[j+1];
                g_numOkosSirens--;
                if (tgt > i) tgt--;
            }
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) { Device_Save(); OkosSiren_PostAssign(addr); }
}

void OkosSiren_Setup(uint16_t addr)
{
    pthread_mutex_lock(&g_deviceMutex);
    int idx = -1;
    for (int i = 0; i < g_numOkosSirens; i++) if (g_okosSirens[i].shortAddr == addr) { idx = i; break; }
    if (idx < 0)                       { pthread_mutex_unlock(&g_deviceMutex); return; }
    if (g_okosSirens[idx].configured)  { pthread_mutex_unlock(&g_deviceMutex); return; }
    if (!g_okosSirens[idx].hasIeee)
        g_okosSirens[idx].hasIeee = Device_GetDiscoveredIeee(addr, g_okosSirens[idx].ieee);
    bool hasIeee = g_okosSirens[idx].hasIeee;
    uint8_t ep   = g_okosSirens[idx].endpoint;
    if (hasIeee) g_okosSirens[idx].configured = true;
    pthread_mutex_unlock(&g_deviceMutex);

    if (!hasIeee) {
        LOG_DEBUG("OkosSiren 0x%04X: missing IEEE, requesting...\n", addr);
        uint8_t p[4] = { addr&0xFF, (addr>>8)&0xFF, 0x01, 0x00 };
        ZNP_Sreq(0x25, 0x01, p, 4, NULL, 3000);
        return;
    }
    LOG_DEBUG("OkosSiren 0x%04X: writing CIE address on ep 0x%02X\n", addr, ep);
    ZNP_WriteCieAddress(addr, ep, 0x14);
}

void OkosSiren_HandleEnroll(uint16_t addr, uint8_t ep, uint8_t transSeq, uint16_t zoneType)
{
    LOG_DEBUG("OkosSiren 0x%04X: Zone Enroll Request, type=0x%04X\n", addr, zoneType);
    uint8_t zoneId = g_nextZoneId++;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++)
        if (g_okosSirens[i].shortAddr == addr) { g_okosSirens[i].zoneId = zoneId; break; }
    pthread_mutex_unlock(&g_deviceMutex);
    Device_Save();
    ZNP_SendZoneEnrollResponse(addr, ep, transSeq, zoneId);
}

// Send a Tuya DP frame using both Cmd 0x00 (Set Data) and Cmd 0x04 (Write Data)
static void SendTuyaDP(uint16_t addr, uint8_t ep, uint8_t dpId, uint8_t dpType, const uint8_t *val, uint16_t valLen)
{
    uint8_t payload[64];
    uint8_t seq = NextSeq();
    payload[0] = 0x11; // Cluster-specific, client to server
    payload[1] = seq;  // ZCL TransSeq
    payload[2] = 0x00; // Tuya Cmd: Set Data
    payload[3] = 0x00; // Tuya Seq High
    payload[4] = seq;  // Tuya Seq Low
    payload[5] = dpId;
    payload[6] = dpType; // 0x01=Bool, 0x02=Value(4B), 0x04=Enum(1B)
    payload[7] = (valLen >> 8) & 0xFF;
    payload[8] = valLen & 0xFF;
    if (val && valLen > 0) {
        memcpy(&payload[9], val, valLen);
    }
    
    // Some Tuya MCUs respond to 0x00, others to 0x04
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_TUYA_CLUSTER, seq, 0, 30, payload, 9 + valLen);
    
    uint8_t seq2 = NextSeq();
    payload[1] = seq2;
    payload[2] = 0x04; // Tuya Cmd: Write Data
    payload[4] = seq2;
    bool ok = ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_TUYA_CLUSTER, seq2, 0, 30, payload, 9 + valLen);
    
    if (!ok) {
        LOG_WARNING("[OKOS] ZNP rejected Tuya DP %u to 0x%04X\n", dpId, addr);
    } else {
        LOG_DEBUG("[OKOS] Sent DP %u type=0x%02X val=%u to 0x%04X\n", dpId, dpType, val ? val[0] : 0, addr);
    }
}

void OkosSiren_ControlAllDuration(uint8_t warnMode, uint16_t dur)
{
    g_sirenActive = (warnMode != 0);
    pthread_mutex_lock(&g_deviceMutex);
    if (g_numOkosSirens == 0) {
        if (warnMode) LOG_ERROR("No Okos sirens registered.\n");
        pthread_mutex_unlock(&g_deviceMutex); return;
    }
    int n = g_numOkosSirens;
    OKOS_SIREN_T tmp[MAX_OKOS_SIRENS];
    memcpy(tmp, g_okosSirens, sizeof(OKOS_SIREN_T)*n);
    pthread_mutex_unlock(&g_deviceMutex);

    for (int i = 0; i < n; i++) {
        if (Device_IsOffline(tmp[i].shortAddr)) {
            LOG_ERROR("OkosSiren 0x%04X OFFLINE! Skipped.\n", tmp[i].shortAddr);
            continue;
        }
        uint8_t ep     = tmp[i].endpoint;
        uint16_t addr  = tmp[i].shortAddr;
        uint8_t toneId = tmp[i].toneId;
        uint8_t vol    = tmp[i].volume;  // 0=Low 1=Med 2=High

        if (!warnMode) {
            uint8_t off = 0;
            // Stop command for standard Tuya Siren DPs
            SendTuyaDP(addr, ep, 1,   0x01, &off, 1);
            usleep(20000);
            SendTuyaDP(addr, ep, 102, 0x01, &off, 1);
            usleep(20000);
            SendTuyaDP(addr, ep, 13,  0x01, &off, 1);
            usleep(20000);
            LOG_INFO("SUCCESS: OkosSiren 0x%04X OFF.\n", addr);
        } else {
            // toneId is 1-based; Tuya melody DPs expect 0-based enum
            uint8_t toneEnum = (toneId > 0) ? (toneId - 1) : 0;
            uint8_t on = 1;

            // Step 1: Pre-configure Tone, Volume, and Duration using Tuya DPs
            // Sending multiple DP versions to cover different Tuya firmwares
            SendTuyaDP(addr, ep, 104, 0x04, &toneEnum, 1);  usleep(20000);
            SendTuyaDP(addr, ep, 116, 0x04, &vol,      1);  usleep(20000);
            SendTuyaDP(addr, ep, 5,   0x04, &vol,      1);  usleep(20000);
            SendTuyaDP(addr, ep, 21,  0x04, &toneEnum, 1);  usleep(20000);

            // Duration is usually sent as a 4-byte value (big-endian) on DP 103 or 14
            uint8_t dur4[4] = { (uint8_t)(dur >> 24), (uint8_t)(dur >> 16),
                                (uint8_t)(dur >> 8),  (uint8_t)dur };
            SendTuyaDP(addr, ep, 103, 0x02, dur4, 4);       usleep(20000);

            // Step 2: Trigger alarm via Tuya Switch DPs
            SendTuyaDP(addr, ep, 1,   0x01, &on, 1);  usleep(20000);
            SendTuyaDP(addr, ep, 102, 0x01, &on, 1);  usleep(20000);
            SendTuyaDP(addr, ep, 13,  0x01, &on, 1);  usleep(20000);

            LOG_INFO("SUCCESS: OkosSiren 0x%04X ON (tone=%d vol=%d).\n", addr, toneId, vol);
        }
    }
}

void OkosSiren_ControlAll(uint8_t warnMode)    { OkosSiren_ControlAllDuration(warnMode, 240); }

void OkosSiren_Control(uint16_t addr, uint8_t warnMode)
{
    g_sirenActive = (warnMode != 0);
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) {
        if (g_okosSirens[i].shortAddr == addr) {
            uint8_t ep     = g_okosSirens[i].endpoint;
            uint8_t toneId = g_okosSirens[i].toneId;
            uint8_t vol    = g_okosSirens[i].volume;  // 0=Low 1=Med 2=High
            pthread_mutex_unlock(&g_deviceMutex);

            if (Device_IsOffline(addr)) {
                LOG_ERROR("OkosSiren 0x%04X OFFLINE!\n", addr);
                return;
            }

            if (!warnMode) {
                uint8_t off = 0;
                // Stop command for standard Tuya Siren DPs
                SendTuyaDP(addr, ep, 1,   0x01, &off, 1);
                usleep(20000);
                SendTuyaDP(addr, ep, 102, 0x01, &off, 1);
                usleep(20000);
                SendTuyaDP(addr, ep, 13,  0x01, &off, 1);
                usleep(20000);
                LOG_INFO("SUCCESS: OkosSiren 0x%04X OFF.\n", addr);
            } else {
                // toneId is 1-based; Tuya melody DPs expect 0-based enum
                uint8_t toneEnum = (toneId > 0) ? (toneId - 1) : 0;
                uint8_t on = 1;
                uint16_t dur = 240;

                // Step 1: Pre-configure Tone, Volume, and Duration using Tuya DPs
                // Sending multiple DP versions to cover different Tuya firmwares
                SendTuyaDP(addr, ep, 104, 0x04, &toneEnum, 1);  usleep(20000);
                SendTuyaDP(addr, ep, 116, 0x04, &vol,      1);  usleep(20000);
                SendTuyaDP(addr, ep, 5,   0x04, &vol,      1);  usleep(20000);
                SendTuyaDP(addr, ep, 21,  0x04, &toneEnum, 1);  usleep(20000);

                // Duration is usually sent as a 4-byte value (big-endian) on DP 103 or 14
                uint8_t dur4[4] = { (uint8_t)(dur >> 24), (uint8_t)(dur >> 16),
                                    (uint8_t)(dur >> 8),  (uint8_t)dur };
                SendTuyaDP(addr, ep, 103, 0x02, dur4, 4);       usleep(20000);

                // Step 2: Trigger alarm via Tuya Switch DPs
                SendTuyaDP(addr, ep, 1,   0x01, &on, 1);  usleep(20000);
                SendTuyaDP(addr, ep, 102, 0x01, &on, 1);  usleep(20000);
                SendTuyaDP(addr, ep, 13,  0x01, &on, 1);  usleep(20000);

                LOG_INFO("SUCCESS: OkosSiren 0x%04X ON (tone=%d vol=%d).\n", addr, toneId, vol);
            }
            return;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
}

void OkosSiren_SetVolume(uint16_t addr, uint8_t vol)
{
    if (vol > 2) vol = 2;
    bool found = false;
    uint8_t ep = 1;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) {
        if (g_okosSirens[i].shortAddr == addr) {
            g_okosSirens[i].volume = vol;
            ep = g_okosSirens[i].endpoint;
            found = true;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        uint8_t vol4[4] = { 0, 0, 0, vol };
        SendTuyaDP(addr, ep, 116, 0x04, &vol, 1);
        SendTuyaDP(addr, ep, 116, 0x02, vol4, 4);
        SendTuyaDP(addr, ep, 5, 0x04, &vol, 1);
        LOG_INFO("SUCCESS: OkosSiren 0x%04X volume=%d (0=Low, 1=Med, 2=High).\n", addr, vol);
    }
    else LOG_ERROR("OkosSiren 0x%04X not found.\n", addr);
}

uint8_t OkosSiren_GetVolume(uint16_t addr)
{
    uint8_t v = 2;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) if (g_okosSirens[i].shortAddr == addr) { v = g_okosSirens[i].volume; break; }
    pthread_mutex_unlock(&g_deviceMutex);
    return v;
}

void OkosSiren_SetTone(uint16_t addr, uint8_t toneId)
{
    if (toneId < 1 || toneId > 18) toneId = OKOS_TONE_BURGLAR;
    bool found = false;
    uint8_t ep = 1;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) {
        if (g_okosSirens[i].shortAddr == addr) {
            g_okosSirens[i].toneId = toneId;
            ep = g_okosSirens[i].endpoint;
            found = true;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        uint8_t zeroTone = (toneId > 0) ? (toneId - 1) : 0;
        uint8_t toneVal  = toneId;
        uint8_t tone4[4] = { 0, 0, 0, zeroTone };
        SendTuyaDP(addr, ep, 104, 0x04, &zeroTone, 1);
        SendTuyaDP(addr, ep, 104, 0x04, &toneVal, 1);
        SendTuyaDP(addr, ep, 104, 0x02, tone4, 4);
        LOG_INFO("SUCCESS: OkosSiren 0x%04X tone=%d.\n", addr, toneId);
    }
    else LOG_ERROR("OkosSiren 0x%04X not found.\n", addr);
}

uint8_t OkosSiren_GetTone(uint16_t addr)
{
    uint8_t t = OKOS_TONE_BURGLAR;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) if (g_okosSirens[i].shortAddr == addr) { t = g_okosSirens[i].toneId; break; }
    pthread_mutex_unlock(&g_deviceMutex);
    return t;
}

void OkosSiren_SetStrobe(uint16_t addr, uint8_t strobeMode)
{
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++)
        if (g_okosSirens[i].shortAddr == addr) { g_okosSirens[i].strobeMode = strobeMode; break; }
    pthread_mutex_unlock(&g_deviceMutex);
    Device_Save();
}

void OkosSiren_ReadBattery(uint16_t addr)
{
    uint8_t ep = OkosSiren_GetEndpoint(addr);
    if (!ep) { LOG_ERROR("OkosSiren 0x%04X not found.\n", addr); return; }

    pthread_mutex_lock(&g_deviceMutex);
    uint8_t bat = 0xFF;
    for (int i = 0; i < g_numOkosSirens; i++) {
        if (g_okosSirens[i].shortAddr == addr) { bat = g_okosSirens[i].batteryPct; break; }
    }
    pthread_mutex_unlock(&g_deviceMutex);

    if (bat != 0xFF) {
        LOG_INFO("[OKOS_SIREN 0x%04X] Battery: %u%%\n", addr, bat);
    } else {
        LOG_INFO("Querying battery for OkosSiren 0x%04X...\n", addr);
    }
    SendTuyaQuery(addr, ep);
    uint8_t zcl[5] = { 0x00, NextSeq(), 0x00, 0x21, 0x00 };
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_POWER_CLUSTER, zcl[1], 0, 30, zcl, 5);
}

void OkosSiren_ReadEnvironment(uint16_t addr)
{
    uint8_t ep = OkosSiren_GetEndpoint(addr);
    if (!ep) { LOG_ERROR("OkosSiren 0x%04X not found.\n", addr); return; }

    pthread_mutex_lock(&g_deviceMutex);
    int16_t temp = 0x7FFF;
    uint16_t hum = 0xFFFF;
    for (int i = 0; i < g_numOkosSirens; i++) {
        if (g_okosSirens[i].shortAddr == addr) {
            temp = g_okosSirens[i].temperatureCdeg;
            hum  = g_okosSirens[i].humidityHpct;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);

    bool printed = false;
    if (temp != 0x7FFF) {
        LOG_INFO("[OKOS_SIREN 0x%04X] Temperature: %.2f C\n", addr, temp / 100.0f);
        printed = true;
    }
    if (hum != 0xFFFF) {
        LOG_INFO("[OKOS_SIREN 0x%04X] Humidity: %.2f%%\n", addr, hum / 100.0f);
        printed = true;
    }

    if (!printed) {
        LOG_INFO("Querying environment (temperature/humidity) for OkosSiren 0x%04X...\n", addr);
    }

    SendTuyaQuery(addr, ep);
    uint8_t s1 = NextSeq();
    uint8_t tzcl[5] = { 0x00, s1, 0x00, 0x00, 0x00 };
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_TEMP_CLUSTER, s1, 0, 30, tzcl, 5);
    usleep(50000);
    uint8_t s2 = NextSeq();
    uint8_t hzcl[5] = { 0x00, s2, 0x00, 0x00, 0x00 };
    ZNP_AfDataRequestExt(2, addr, ep, 0, 8, OKOS_HUM_CLUSTER, s2, 0, 30, hzcl, 5);
}

bool OkosSiren_IsKnown(uint16_t addr)
{
    bool r = false;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) if (g_okosSirens[i].shortAddr == addr) { r = true; break; }
    pthread_mutex_unlock(&g_deviceMutex);
    return r;
}

uint8_t OkosSiren_GetEndpoint(uint16_t addr)
{
    uint8_t ep = 0;
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) if (g_okosSirens[i].shortAddr == addr) { ep = g_okosSirens[i].endpoint; break; }
    pthread_mutex_unlock(&g_deviceMutex);
    return ep;
}

void OkosSiren_UpdateSeen(uint16_t addr)
{
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numOkosSirens; i++) if (g_okosSirens[i].shortAddr == addr) { g_okosSirens[i].lastSeen = ZNP_GetCurrentTime(); break; }
    pthread_mutex_unlock(&g_deviceMutex);
}

void OkosSiren_PrintStatus(void)
{
    pthread_mutex_lock(&g_deviceMutex);
    LOG_RAW("Okos Sirens (%d):\n", g_numOkosSirens);
    double now = ZNP_GetCurrentTime();
    for (int i = 0; i < g_numOkosSirens; i++) {
        OKOS_SIREN_T *s = &g_okosSirens[i];
        LOG_RAW("  0x%04X ep=0x%02X tone=%d vol=%d tamper=%s seen=%.1fs ago",
               s->shortAddr, s->endpoint, s->toneId, s->volume,
               s->isTampered ? "YES" : "no", now - s->lastSeen);
        if (s->temperatureCdeg != 0x7FFF) LOG_RAW(" temp=%.2fC", s->temperatureCdeg/100.0f);
        if (s->humidityHpct   != 0xFFFF) LOG_RAW(" hum=%.2f%%", s->humidityHpct/100.0f);
        if (s->batteryPct     != 0xFF)   LOG_RAW(" bat=%u%%", s->batteryPct);
        LOG_RAW("\n");
    }
    pthread_mutex_unlock(&g_deviceMutex);
}

void OkosSiren_PollAll(void)
{
    static double lastPoll = 0;
    double now = ZNP_GetCurrentTime();
    if (now - lastPoll < 60.0) return;
    lastPoll = now;

    pthread_mutex_lock(&g_deviceMutex);
    int n = g_numOkosSirens;
    uint16_t addrs[MAX_OKOS_SIRENS]; uint8_t eps[MAX_OKOS_SIRENS];
    for (int i = 0; i < n; i++) { addrs[i] = g_okosSirens[i].shortAddr; eps[i] = g_okosSirens[i].endpoint; }
    pthread_mutex_unlock(&g_deviceMutex);

    static uint8_t seq = 100;
    for (int i = 0; i < n; i++) {
        uint8_t zcl[5] = { 0x00, ++seq, 0x00, 0x00, 0x00 };
        ZNP_AfDataRequestExt(0x02, addrs[i], eps[i], 0, 8, 0x0000, seq, 0x00, 30, zcl, 5);
        usleep(50000);
    }
}

void OkosSiren_DiscoverAllActiveEp(void)
{
    pthread_mutex_lock(&g_deviceMutex);
    int n = g_numOkosSirens; uint16_t addrs[MAX_OKOS_SIRENS];
    for (int i = 0; i < n; i++) addrs[i] = g_okosSirens[i].shortAddr;
    pthread_mutex_unlock(&g_deviceMutex);
    for (int i = 0; i < n; i++) { ZNP_ZdoActiveEpReq(addrs[i]); ZNP_QuerySimpleDesc(addrs[i], 1); }
}
#endif // ENABLE_OKOS_SIREN
