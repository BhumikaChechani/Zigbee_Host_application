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
    if (cmdId == 0x01) // Zone Enroll Request
    {
      if (zclLen >= 2) {
        uint16_t zoneType = zcl[0] | (zcl[1] << 8);
        uint8_t transSeq = af_->data[hdrLen - 2];
        ContactSensor_HandleEnroll(af_->srcAddr, af_->srcEp, transSeq,
                                   zoneType);
      }
    } else if (cmdId == 0x00) // Zone Status Change Notification
    {
      if (zclLen >= 4) {
        uint16_t zoneStatus = zcl[0] | (zcl[1] << 8);
        uint8_t zoneId = zcl[3];
        ContactSensor_HandleStatus(af_->srcAddr, zoneStatus, zoneId);
      }
    }
  }
}

static void *ContactSensor_Thread(void *arg_) {
  (void)arg_;
  while (1) {
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop(&s_contactInbox);
    if (msg == NULL)
      continue;

    if (msg->kind == SENSOR_MSG_ASSIGN) {
      ContactSensor_Setup(msg->shortAddr);
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
  pthread_mutex_lock(&g_deviceMutex);
  int idx = -1;
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      idx = i;
      break;
    }
  }

  bool changed = false;
  if (idx == -1) {
    if (g_numContactSensors < MAX_CONTACT_SENSORS) {
      printf(" Contact Sensor discovered: short=0x%04X, ep=0x%02X\n",
             shortAddr_, endpoint_);
      g_contactSensors[g_numContactSensors].shortAddr = shortAddr_;
      g_contactSensors[g_numContactSensors].endpoint = endpoint_;
      g_contactSensors[g_numContactSensors].lastSeen = ZNP_GetCurrentTime();
      g_contactSensors[g_numContactSensors].zoneId = -1;
      g_contactSensors[g_numContactSensors].hasIeee = Device_GetDiscoveredIeee(
          shortAddr_, g_contactSensors[g_numContactSensors].ieee);
      g_contactSensors[g_numContactSensors].configured = false;
      g_numContactSensors++;
      changed = true;
    }
  } else {
    if (g_contactSensors[idx].endpoint != endpoint_) {
      g_contactSensors[idx].endpoint = endpoint_;
      changed = true;
    }
    g_contactSensors[idx].lastSeen = ZNP_GetCurrentTime();
    if (!g_contactSensors[idx].hasIeee) {
      g_contactSensors[idx].hasIeee =
          Device_GetDiscoveredIeee(shortAddr_, g_contactSensors[idx].ieee);
      if (g_contactSensors[idx].hasIeee)
        changed = true;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);

  if (changed)
    Device_Save();
  ContactSensor_PostAssign(shortAddr_);
}

void ContactSensor_UpdateIeee(uint16_t shortAddr_, const uint8_t *ieee_) {
  bool found = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      memcpy(g_contactSensors[i].ieee, ieee_, 8);
      g_contactSensors[i].hasIeee = true;
      found = true;
      break;
    }
  }
  if (found) {
    for (int i = g_numContactSensors - 1; i >= 0; i--) {
      if (g_contactSensors[i].shortAddr != shortAddr_ &&
          g_contactSensors[i].hasIeee &&
          memcmp(g_contactSensors[i].ieee, ieee_, 8) == 0) {
        for (int j = i; j < g_numContactSensors - 1; j++) {
          g_contactSensors[j] = g_contactSensors[j + 1];
        }
        g_numContactSensors--;
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
    printf("Contact sensor 0x%04X missing IEEE - requesting...\n", shortAddr_);
    uint8_t reqPay[4] = {shortAddr_ & 0xFF, (shortAddr_ >> 8) & 0xFF, 0x01,
                         0x00};
    ZNP_Sreq(0x25, 0x01, reqPay, 4, NULL, 3000);
    return;
  }

  uint8_t sensorIeee[8];
  uint8_t endpoint = g_contactSensors[idx].endpoint;
  memcpy(sensorIeee, g_contactSensors[idx].ieee, 8);
  g_contactSensors[idx].configured = true;
  pthread_mutex_unlock(&g_deviceMutex);

  printf("Configuring Contact Sensor 0x%04X...\n", shortAddr_);

  // 1. Bind IAS Zone cluster (0x0500)
  ZNP_ZdoBindReq(shortAddr_, sensorIeee, endpoint, 0x0500, g_coordinatorIeee,
                 8);
  usleep(500000);

  // 2. Write coordinator's IEEE to sensor's IAS_CIE_Address attribute (0x0010).
  ZNP_WriteCieAddress(shortAddr_, endpoint, 0x20);
  usleep(500000);

  // 3. Force send a Zone Enroll Response in case it doesn't send a Request
  ZNP_SendZoneEnrollResponse(shortAddr_, endpoint, 0x21, 0x01);
  usleep(200000);

  printf("Configuration sent to Contact Sensor 0x%04X!\n", shortAddr_);
}

void ContactSensor_HandleEnroll(uint16_t shortAddr_, uint8_t endpoint_,
                                uint8_t transSeq_, uint16_t zoneType_) {
  printf("   -> Zone Enroll Request from Contact Sensor 0x%04X, "
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
  printf("   -> Zone Status Change from Contact Sensor 0x%04X: "
         "zone_status=0x%04X, zone_id=%d\n",
         shortAddr_, zoneStatus_, zoneId_);

  bool open = (zoneStatus_ & 0x0001) != 0;
  if (open) {
    UseCase_Post(UC_CONTACT_OPEN, shortAddr_, zoneStatus_);
  } else {
    UseCase_Post(UC_CONTACT_CLOSED, shortAddr_, zoneStatus_);
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
    printf(", ep=0x%02X, zone_id=%d, last_seen=%.1fs ago\n",
           g_contactSensors[i].endpoint, g_contactSensors[i].zoneId,
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
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].shortAddr == shortAddr_) {
      g_contactSensors[i].lastSeen = ZNP_GetCurrentTime();
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
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
#endif
