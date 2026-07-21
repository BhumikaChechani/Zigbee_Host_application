#include "config.h"
#if ENABLE_CONTACT_SENSOR
#include "contact_sensor.h"
#include "msg_queue.h"
#include "usecase.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

CONTACT_SENSOR_T g_contactSensors[MAX_CONTACT_SENSORS];
int g_numContactSensors = 0;

static MSG_QUEUE_T s_contactInbox;
static pthread_t s_contactThread;

static void ContactSensor_HandleAf(const AF_MSG_T *af_) {
  if (af_->dataLen < 3)
    return;
  uint8_t fc = af_->data[0];
  int hdrLen = (fc & 0x04) ? 5 : 3;
  if (af_->dataLen < hdrLen)
    return;

  uint8_t cmdId = af_->data[hdrLen - 1];

  if (af_->clusterId == 0x0500) {
    const uint8_t *zcl = &af_->data[hdrLen];
    int zclLen = af_->dataLen - hdrLen;
    // Frame type matters here: cluster-specific cmd 0x01 is Zone Enroll
    // Request, but GLOBAL cmd 0x01 is a Read Attributes Response (our active
    // zone-status refresh) - they must not be confused.
    bool clusterSpecific = ((fc & 0x03) == 0x01);
    if (clusterSpecific && cmdId == 0x01) // Zone Enroll Request
    {
      if (zclLen >= 2) {
        uint16_t zoneType = zcl[0] | (zcl[1] << 8);
        uint8_t transSeq = af_->data[hdrLen - 2];
        ContactSensor_HandleEnroll(af_->srcAddr, af_->srcEp, transSeq,
                                   zoneType);
      }
    } else if (clusterSpecific && cmdId == 0x00) // Zone Status Change Notification
    {
      if (zclLen >= 4) {
        uint16_t zoneStatus = zcl[0] | (zcl[1] << 8);
        uint8_t zoneId = zcl[3];
        ContactSensor_HandleStatus(af_->srcAddr, zoneStatus, zoneId);
      }
    } else if (!clusterSpecific && cmdId == 0x01) // Read Attr Response (status refresh)
    {
      // AttrID(2)=0x0002 + Status(1)=0 + Type(1)=0x19 map16 + Value(2)
      if (zclLen >= 6 && zcl[0] == 0x02 && zcl[1] == 0x00 && zcl[2] == 0x00) {
        uint16_t zoneStatus = zcl[4] | (zcl[5] << 8);
        bool open = (zoneStatus & 0x0001) != 0;
        pthread_mutex_lock(&g_deviceMutex);
        for (int i = 0; i < g_numContactSensors; i++) {
          if (g_contactSensors[i].shortAddr == af_->srcAddr) {
            double now = ZNP_GetCurrentTime();
            if (g_contactSensors[i].isOpen != open) {
              LOG_DEBUG("Contact Sensor 0x%04X state corrected by refresh: %s "
                     "(a change notification was missed)\n",
                     af_->srcAddr, open ? "OPEN" : "CLOSED");
              if (open)
                g_contactSensors[i].lastOpenedTime = now;
            }
            g_contactSensors[i].isOpen = open;
            g_contactSensors[i].lastStatusTime = now;
            break;
          }
        }
        pthread_mutex_unlock(&g_deviceMutex);
      }
    }
  } else if (af_->clusterId == 0x0001) { // Power Configuration
    if (cmdId == 0x01) { // Read Attributes Response
      const uint8_t *zcl = &af_->data[hdrLen];
      int zclLen = af_->dataLen - hdrLen;
      if (zclLen >= 5 && zcl[0] == 0x20 && zcl[1] == 0x00 && zcl[2] == 0x00) { // Success
        uint8_t bat = zcl[4]; // Unit is 100 mV
        LOG_DEBUG("Contact Sensor 0x%04X Battery Voltage: %.1f V\n", af_->srcAddr, (float)bat / 10.0);
      }
    }
  } else if (af_->clusterId == 0x0402) { // Temperature Measurement
    if (cmdId == 0x01 || cmdId == 0x0A) { // Read Attributes Response or Report Attributes
      const uint8_t *zcl = &af_->data[hdrLen];
      int zclLen = af_->dataLen - hdrLen;
      if (cmdId == 0x01 && zclLen >= 6 && zcl[0] == 0x00 && zcl[1] == 0x00 && zcl[2] == 0x00) { // Read Resp Success
        int16_t temp = (int16_t)(zcl[4] | (zcl[5] << 8));
        LOG_DEBUG("Contact Sensor 0x%04X Temperature: %.2f °C\n", af_->srcAddr, (float)temp / 100.0);
      } else if (cmdId == 0x0A && zclLen >= 5 && zcl[0] == 0x00 && zcl[1] == 0x00) { // Report
        int16_t temp = (int16_t)(zcl[3] | (zcl[4] << 8));
        LOG_DEBUG("Contact Sensor 0x%04X Temperature Report: %.2f °C\n", af_->srcAddr, (float)temp / 100.0);
      }
    }
  }

  if ((fc & 0x10) == 0 && cmdId != 0x0B) {
    uint8_t transSeq = af_->data[1];
    ZNP_SendDefaultResponse(af_->srcAddr, af_->srcEp, af_->clusterId, transSeq, cmdId, 0x00);
  }
}

// Actively read the IAS ZoneStatus attribute (0x0500 / 0x0002) so a missed
// Zone Status Change Notification cannot leave isOpen stale forever. The
// response is parsed in ContactSensor_HandleAf (global Read Attr Response).
static void ContactSensor_SendStatusRead(uint16_t shortAddr_) {
  uint8_t endpoint = 1;
  bool found = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      endpoint = g_contactSensors[i].endpoint;
      found = true;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  if (!found)
    return;

  static uint8_t s_refreshSeq = 0xE0;
  uint8_t seq = __atomic_add_fetch(&s_refreshSeq, 1, __ATOMIC_RELAXED);
  uint8_t zcl[5];
  zcl[0] = 0x00; // FC: global, client->server
  zcl[1] = seq;
  zcl[2] = 0x00; // Read Attributes
  zcl[3] = 0x02; // Attr 0x0002 (ZoneStatus)
  zcl[4] = 0x00;
  ZNP_AfDataRequestExt(2, shortAddr_, endpoint, 0, 8, 0x0500, seq, 0, 30, zcl, 5);
}

static void *ContactSensor_Thread(void *arg_) {
  (void)arg_;
  while (1) {
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop(&s_contactInbox);
    if (msg == NULL)
      continue;

    if (msg->kind == SENSOR_MSG_ASSIGN) {
      ContactSensor_Setup(msg->shortAddr);
    } else if (msg->kind == SENSOR_MSG_REFRESH) {
      ContactSensor_SendStatusRead(msg->shortAddr);
    } else if (msg->kind == SENSOR_MSG_AF) {
      ContactSensor_HandleAf(&msg->af);
    }
    free(msg);
  }
  return NULL;
}

void ContactSensor_Init(void) {
  memset(g_contactSensors, 0, sizeof(g_contactSensors));
  g_numContactSensors = 0;
  MsgQueue_Init(&s_contactInbox);
}

void ContactSensor_Start(void) {
  pthread_create(&s_contactThread, NULL, ContactSensor_Thread, NULL);
}

void ContactSensor_PostAssign(uint16_t shortAddr_) {
  SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc(1, sizeof(SENSOR_MSG_T));
  if (msg == NULL)
    return;
  msg->kind = SENSOR_MSG_ASSIGN;
  msg->shortAddr = shortAddr_;
  MsgQueue_Push(&s_contactInbox, msg);
}

void ContactSensor_PostAf(uint16_t shortAddr_, const AF_MSG_T *af_) {
  SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc(1, sizeof(SENSOR_MSG_T));
  if (msg == NULL)
    return;
  msg->kind = SENSOR_MSG_AF;
  msg->shortAddr = shortAddr_;
  msg->af = *af_;
  MsgQueue_Push(&s_contactInbox, msg);
}

void ContactSensor_Discover(uint16_t shortAddr_, uint8_t endpoint_) {
  // Battery sensors frequently rejoin with a brand-new short address, so the
  // IEEE - not the short address - is the stable identity. Matching on it
  // updates the existing entry (preserving zoneId and isOpen) instead of
  // accumulating phantom duplicates that break the door/zone lookup.
  uint8_t ieee[8];
  bool haveIeee = Device_GetDiscoveredIeee(shortAddr_, ieee);

  pthread_mutex_lock(&g_deviceMutex);
  int idx = -1;
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      idx = i;
      break;
    }
    if (haveIeee && g_contactSensors[i].hasIeee &&
        memcmp(g_contactSensors[i].ieee, ieee, 8) == 0) {
      idx = i;
      break;
    }
  }

  bool changed = false;
  bool isRejoin = false;
  if (idx == -1) {
    if (g_numContactSensors < MAX_CONTACT_SENSORS) {
      LOG_DEBUG("Contact Sensor discovered: short=0x%04X, ep=0x%02X\n",
             shortAddr_, endpoint_);
      LOG_EVENT("CONTACT", shortAddr_, "Network Join\n");
      g_contactSensors[g_numContactSensors].shortAddr = shortAddr_;
      g_contactSensors[g_numContactSensors].endpoint = endpoint_;
      g_contactSensors[g_numContactSensors].lastSeen = ZNP_GetCurrentTime();
      g_contactSensors[g_numContactSensors].zoneId = -1;
      g_contactSensors[g_numContactSensors].hasIeee = haveIeee;
      if (haveIeee) {
        memcpy(g_contactSensors[g_numContactSensors].ieee, ieee, 8);
      }
      g_contactSensors[g_numContactSensors].configured = false;
      g_contactSensors[g_numContactSensors].isOpen = false;
      g_contactSensors[g_numContactSensors].isTampered = false;
      g_contactSensors[g_numContactSensors].setupRetries = 0;
      g_contactSensors[g_numContactSensors].lastSetupAttempt = 0.0;
      g_numContactSensors++;
      changed = true;
    }
  } else {
    if (g_contactSensors[idx].shortAddr != shortAddr_) {
      LOG_DEBUG("Contact Sensor 0x%04X rejoined as 0x%04X (same IEEE) - reusing entry\n",
             g_contactSensors[idx].shortAddr, shortAddr_);
      LOG_EVENT("CONTACT", shortAddr_, "Network Rejoin\n");
      g_contactSensors[idx].shortAddr = shortAddr_;
      isRejoin = true;
      changed = true;
    }
    if (g_contactSensors[idx].endpoint != endpoint_) {
      g_contactSensors[idx].endpoint = endpoint_;
      changed = true;
    }
    g_contactSensors[idx].lastSeen = ZNP_GetCurrentTime();
    if (!g_contactSensors[idx].hasIeee && haveIeee) {
      memcpy(g_contactSensors[idx].ieee, ieee, 8);
      g_contactSensors[idx].hasIeee = true;
      changed = true;
    }
  }

  // Device-side bindings target the coordinator's (stable) IEEE, so they
  // survive a rejoin; skip the setup flood for a pure rejoin of an
  // already-configured sensor.
  bool alreadyConfigured = (idx != -1 && g_contactSensors[idx].configured);
  pthread_mutex_unlock(&g_deviceMutex);

  if (changed)
    Device_Save();
  if (!(isRejoin && alreadyConfigured))
    ContactSensor_PostAssign(shortAddr_);
}

void ContactSensor_UpdateIeee(uint16_t shortAddr_, const uint8_t *ieee_) {
  bool found = false;
  pthread_mutex_lock(&g_deviceMutex);
  int targetIdx = -1;
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      memcpy(g_contactSensors[i].ieee, ieee_, 8);
      g_contactSensors[i].hasIeee = true;
      targetIdx = i;
      found = true;
      break;
    }
  }
  if (found) {
    for (int i = g_numContactSensors - 1; i >= 0; i--) {
      if (g_contactSensors[i].shortAddr != shortAddr_ &&
          g_contactSensors[i].hasIeee &&
          memcmp(g_contactSensors[i].ieee, ieee_, 8) == 0) {
        
        // Preserve configuration from the old (now stale) entry
        g_contactSensors[targetIdx].zoneId = g_contactSensors[i].zoneId;
        g_contactSensors[targetIdx].configured = g_contactSensors[i].configured;
        g_contactSensors[targetIdx].isOpen = g_contactSensors[i].isOpen;
        g_contactSensors[targetIdx].lastOpenedTime = g_contactSensors[i].lastOpenedTime;

        for (int j = i; j < g_numContactSensors - 1; j++) {
          g_contactSensors[j] = g_contactSensors[j + 1];
        }
        g_numContactSensors--;

        if (targetIdx > i) {
            targetIdx--;
        }
      }
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  if (found) {
    Device_Save();
    ContactSensor_PostAssign(shortAddr_);
  }
}

void ContactSensor_Setup(uint16_t shortAddr_) {
  pthread_mutex_lock(&g_deviceMutex);
  int idx = -1;
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      idx = i;
      break;
    }
  }
  if (idx == -1 || g_contactSensors[idx].configured) {
    pthread_mutex_unlock(&g_deviceMutex);
    return;
  }
  if (!g_contactSensors[idx].hasIeee) {
    g_contactSensors[idx].hasIeee =
        Device_GetDiscoveredIeee(shortAddr_, g_contactSensors[idx].ieee);
  }
  if (!g_contactSensors[idx].hasIeee) {
    pthread_mutex_unlock(&g_deviceMutex);
    LOG_DEBUG("Contact sensor 0x%04X missing IEEE - requesting...\n", shortAddr_);
    uint8_t reqPay[4] = {shortAddr_ & 0xFF, (shortAddr_ >> 8) & 0xFF, 0x01,
                         0x00};
    ZNP_Sreq(0x25, 0x01, reqPay, 4, NULL, 3000);
    return;
  }

  uint8_t sensorIeee[8];
  uint8_t endpoint = g_contactSensors[idx].endpoint;
  memcpy(sensorIeee, g_contactSensors[idx].ieee, 8);
  g_contactSensors[idx].lastSetupAttempt = ZNP_GetCurrentTime();
  pthread_mutex_unlock(&g_deviceMutex);

  LOG_DEBUG("Configuring Contact Sensor 0x%04X...\n", shortAddr_);

  // 1. Bind IAS Zone cluster (0x0500)
  bool bindOk = ZNP_ZdoBindReq(shortAddr_, sensorIeee, endpoint, 0x0500,
                               g_coordinatorIeee, 8);
  usleep(500000);

  // 2. Write coordinator's IEEE to sensor's IAS_CIE_Address attribute (0x0010).
  bool cieOk = ZNP_WriteCieAddress(shortAddr_, endpoint, 0x20);
  usleep(500000);

  // 3. Force send a Zone Enroll Response in case it doesn't send a Request
  ZNP_SendZoneEnrollResponse(shortAddr_, endpoint, 0x21, 0x01);
  usleep(200000);

  // Only mark configured when the critical steps were accepted; otherwise
  // leave it unconfigured so the next frame from the device retriggers setup.
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      if (bindOk && cieOk) {
        g_contactSensors[i].configured = true;
        g_contactSensors[i].setupRetries = 0;
      } else {
        g_contactSensors[i].setupRetries++;
        LOG_ERROR("Contact Sensor 0x%04X setup FAILED (bind=%s cie=%s, attempt %u) - will retry\n",
               shortAddr_, bindOk ? "OK" : "FAIL", cieOk ? "OK" : "FAIL",
               g_contactSensors[i].setupRetries);
      }
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);

  if (bindOk && cieOk) {
    LOG_DEBUG("Configuration sent to Contact Sensor 0x%04X!\n", shortAddr_);
  }
}

void ContactSensor_HandleEnroll(uint16_t shortAddr_, uint8_t endpoint_,
                                uint8_t transSeq_, uint16_t zoneType_) {
  LOG_DEBUG("-> Zone Enroll Request from Contact Sensor 0x%04X, "
         "zone_type=0x%04X\n",
         shortAddr_, zoneType_);
  uint8_t zoneId = g_nextZoneId++;

  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      g_contactSensors[i].zoneId = zoneId;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  Device_Save();

  ZNP_SendZoneEnrollResponse(shortAddr_, endpoint_, transSeq_, zoneId);
}

void ContactSensor_HandleStatus(uint16_t shortAddr_, uint16_t zoneStatus_,
                                uint8_t zoneId_) {
  LOG_DEBUG("-> Zone Status Change from Contact Sensor 0x%04X: "
         "zone_status=0x%04X, zone_id=%d\n",
         shortAddr_, zoneStatus_, zoneId_);

  bool open = (zoneStatus_ & 0x0001) != 0;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      double now = ZNP_GetCurrentTime();
      if (open)
        g_contactSensors[i].lastOpenedTime = now;
      g_contactSensors[i].isOpen = open;
      g_contactSensors[i].lastStatusTime = now;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);

  if (open) {
    UseCase_Post(UC_CONTACT_OPEN, shortAddr_, zoneStatus_, 0);
  } else {
    UseCase_Post(UC_CONTACT_CLOSED, shortAddr_, zoneStatus_, 0);
  }

  bool tamper_active = (zoneStatus_ & 0x0004) != 0; // Tamper bit

  pthread_mutex_lock(&g_deviceMutex);
  bool tamperChanged = false;
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      if (g_contactSensors[i].isTampered != tamper_active) {
        g_contactSensors[i].isTampered = tamper_active;
        tamperChanged = true;
      }
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);

  if (tamperChanged) {
    if (tamper_active) {
      UseCase_Post(UC_TAMPER_DETECTED, shortAddr_, zoneStatus_, 0);
    } else {
      UseCase_Post(UC_TAMPER_CLEARED, shortAddr_, 0, 0);
    }
  }
}

void ContactSensor_PrintStatus(void) {
  pthread_mutex_lock(&g_deviceMutex);
  printf("Registered Contact Sensors (%d):\n", g_numContactSensors);
  double now = ZNP_GetCurrentTime();
  for (int i = 0; i < g_numContactSensors; i++) {
    printf("  - 0x%04X: IEEE=", g_contactSensors[i].shortAddr);
    if (g_contactSensors[i].hasIeee) {
      for (int j = 7; j >= 0; j--) {
        printf("%02x", g_contactSensors[i].ieee[j]);
      }
    } else {
      printf("Unknown");
    }
    printf(", ep=0x%02X, zone_id=%d, state=%s, last_seen=%.1fs ago\n",
           g_contactSensors[i].endpoint, g_contactSensors[i].zoneId,
           g_contactSensors[i].isOpen ? "OPEN" : "CLOSED",
           now - g_contactSensors[i].lastSeen);
  }
  pthread_mutex_unlock(&g_deviceMutex);
}

bool ContactSensor_IsKnown(uint16_t shortAddr_) {
  bool known = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      known = true;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  return known;
}

void ContactSensor_UpdateSeen(uint16_t shortAddr_) {
  bool retrySetup = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      double now = ZNP_GetCurrentTime();
      g_contactSensors[i].lastSeen = now;
      // The device is provably awake right now: if setup never succeeded,
      // this is the best moment to retry it (paced, capped).
      if (!g_contactSensors[i].configured && g_contactSensors[i].setupRetries < 5 &&
          now - g_contactSensors[i].lastSetupAttempt > 30.0) {
        retrySetup = true;
      }
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  if (retrySetup) {
    LOG_DEBUG("Contact Sensor 0x%04X is awake and unconfigured - retrying setup\n", shortAddr_);
    ContactSensor_PostAssign(shortAddr_);
  }
}

void ContactSensor_DiscoverAllActiveEp(void) {
  pthread_mutex_lock(&g_deviceMutex);
  int tempNum = g_numContactSensors;
  uint16_t tempAddrs[MAX_CONTACT_SENSORS];
  for (int i = 0; i < g_numContactSensors; i++) {
    tempAddrs[i] = g_contactSensors[i].shortAddr;
  }
  pthread_mutex_unlock(&g_deviceMutex);

  for (int i = 0; i < tempNum; i++) {
    ZNP_ZdoActiveEpReq(tempAddrs[i]);
    ZNP_QuerySimpleDesc(tempAddrs[i], 1);
  }
}

void ContactSensor_PostRefresh(uint16_t shortAddr_) {
  SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc(1, sizeof(SENSOR_MSG_T));
  if (msg == NULL)
    return;
  msg->kind = SENSOR_MSG_REFRESH;
  msg->shortAddr = shortAddr_;
  MsgQueue_Push(&s_contactInbox, msg);
}

void ContactSensor_RefreshIfStale(double maxAgeSeconds_) {
  uint16_t staleAddrs[MAX_CONTACT_SENSORS];
  int numStale = 0;
  double now = ZNP_GetCurrentTime();

  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (!g_contactSensors[i].configured)
      continue;
    if (now - g_contactSensors[i].lastStatusTime < maxAgeSeconds_)
      continue;
    // Rate-limit: occupancy events can arrive several times per second.
    if (now - g_contactSensors[i].lastRefreshReq < 10.0)
      continue;
    g_contactSensors[i].lastRefreshReq = now;
    staleAddrs[numStale++] = g_contactSensors[i].shortAddr;
  }
  pthread_mutex_unlock(&g_deviceMutex);

  for (int i = 0; i < numStale; i++) {
    ContactSensor_PostRefresh(staleAddrs[i]);
  }
}

void ContactSensor_ReadEnvironment(uint16_t shortAddr_) {
  LOG_DEBUG("Requesting Environment Data (Battery & Temp) from Contact Sensor 0x%04X...\n", shortAddr_);
  
  // Read Battery Voltage (Cluster 0x0001, Attr 0x0020) on EP 1
  uint8_t zclFrameBat[5];
  zclFrameBat[0] = 0x00; 
  zclFrameBat[1] = 0xCF;
  zclFrameBat[2] = 0x00; // Read Attributes
  zclFrameBat[3] = 0x20; // Attr 0x0020
  zclFrameBat[4] = 0x00; 
  ZNP_AfDataRequestExt(2, shortAddr_, 1, 0, 8, 0x0001, 0xCF, 0, 30, zclFrameBat, 5);
  
  usleep(200000); // Wait 200ms
  
  // Read Temperature (Cluster 0x0402, Attr 0x0000) on EP 1
  uint8_t zclFrameTemp[5];
  zclFrameTemp[0] = 0x00; 
  zclFrameTemp[1] = 0xD0;
  zclFrameTemp[2] = 0x00; // Read Attributes
  zclFrameTemp[3] = 0x00; // Attr 0x0000
  zclFrameTemp[4] = 0x00; 
  ZNP_AfDataRequestExt(2, shortAddr_, 1, 0, 8, 0x0402, 0xD0, 0, 30, zclFrameTemp, 5);
}
#endif
