#include "config.h"
#if ENABLE_VIBRATION_SENSOR
#include "vibration_sensor.h"
#include "msg_queue.h"
#include "usecase.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

VIBRATION_SENSOR_T g_vibrationSensors[MAX_VIBRATION_SENSORS];
int g_numVibrationSensors = 0;

static MSG_QUEUE_T s_vibrationInbox;
static pthread_t s_vibrationThread;

static void VibrationSensor_HandleAf(const AF_MSG_T *af_) {
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
        VibrationSensor_HandleEnroll(af_->srcAddr, af_->srcEp, transSeq, zoneType);
      }
    } else if (cmdId == 0x00) // Zone Status Change Notification
    {
      if (zclLen >= 4) {
        uint16_t zoneStatus = zcl[0] | (zcl[1] << 8);
        uint8_t zoneId = zcl[3];
        uint8_t transSeq = af_->data[hdrLen - 2];
        VibrationSensor_HandleStatus(af_->srcAddr, zoneStatus, zoneId);
        // The WISZB-13x sensor requires a Default Response, otherwise it never clears its alarm bits
        ZNP_SendDefaultResponse(af_->srcAddr, af_->srcEp, 0x0500, transSeq, cmdId, 0x00);
      }
    }
  } else if (af_->clusterId == 0x0012) { // Multistate Input (Aqara specific actions)
    // Multistate Input typically sends Report Attributes (cmdId == 0x0A)
    // The attribute is usually Present Value (0x0055)
    // We just print the payload so the user can see it's not hardcoded!
    const uint8_t *zcl = &af_->data[hdrLen];
    int zclLen = af_->dataLen - hdrLen;
    printf("   -> [Aqara Vibration Action] from 0x%04X, cluster 0x0012: ", af_->srcAddr);
    for (int i = 0; i < zclLen; i++) printf("%02X ", zcl[i]);
    printf("\n");
  } else if (af_->clusterId == 0x0000) { // Basic cluster (Aqara custom attributes)
    const uint8_t *zcl = &af_->data[hdrLen];
    int zclLen = af_->dataLen - hdrLen;
    printf("   -> [Aqara Vibration Basic] from 0x%04X, cluster 0x0000: ", af_->srcAddr);
    for (int i = 0; i < zclLen; i++) printf("%02X ", zcl[i]);
    printf("\n");
  }
}

static void *VibrationSensor_Thread(void *arg_) {
  (void)arg_;
  while (1) {
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop(&s_vibrationInbox);
    if (msg == NULL)
      continue;

    if (msg->kind == SENSOR_MSG_ASSIGN) {
      VibrationSensor_Setup(msg->shortAddr);
    } else if (msg->kind == SENSOR_MSG_AF) {
      VibrationSensor_HandleAf(&msg->af);
    }
    free(msg);
  }
  return NULL;
}

void VibrationSensor_Init(void) {
  memset(g_vibrationSensors, 0, sizeof(g_vibrationSensors));
  g_numVibrationSensors = 0;
  MsgQueue_Init(&s_vibrationInbox);
}

void VibrationSensor_Start(void) {
  pthread_create(&s_vibrationThread, NULL, VibrationSensor_Thread, NULL);
}

void VibrationSensor_PostAssign(uint16_t shortAddr_) {
  SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc(1, sizeof(SENSOR_MSG_T));
  if (msg == NULL)
    return;
  msg->kind = SENSOR_MSG_ASSIGN;
  msg->shortAddr = shortAddr_;
  MsgQueue_Push(&s_vibrationInbox, msg);
}

void VibrationSensor_PostAf(uint16_t shortAddr_, const AF_MSG_T *af_) {
  SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc(1, sizeof(SENSOR_MSG_T));
  if (msg == NULL)
    return;
  msg->kind = SENSOR_MSG_AF;
  msg->shortAddr = shortAddr_;
  msg->af = *af_;
  MsgQueue_Push(&s_vibrationInbox, msg);
}

void VibrationSensor_Discover(uint16_t shortAddr_, uint8_t endpoint_) {
  pthread_mutex_lock(&g_deviceMutex);
  int idx = -1;
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      idx = i;
      break;
    }
  }

  bool changed = false;
  if (idx == -1) {
    if (g_numVibrationSensors < MAX_VIBRATION_SENSORS) {
      printf(" Vibration Sensor discovered: short=0x%04X, ep=0x%02X\n", shortAddr_, endpoint_);
      g_vibrationSensors[g_numVibrationSensors].shortAddr = shortAddr_;
      g_vibrationSensors[g_numVibrationSensors].endpoint = endpoint_;
      g_vibrationSensors[g_numVibrationSensors].lastSeen = ZNP_GetCurrentTime();
      g_vibrationSensors[g_numVibrationSensors].zoneId = -1;
      g_vibrationSensors[g_numVibrationSensors].hasIeee = Device_GetDiscoveredIeee(shortAddr_, g_vibrationSensors[g_numVibrationSensors].ieee);
      g_vibrationSensors[g_numVibrationSensors].configured = false;
      g_vibrationSensors[g_numVibrationSensors].isVibrating = false;
      g_vibrationSensors[g_numVibrationSensors].lastVibrationTime = 0.0;
      g_vibrationSensors[g_numVibrationSensors].isMoving = false;
      g_vibrationSensors[g_numVibrationSensors].lastMovementTime = 0.0;
      g_numVibrationSensors++;
      changed = true;
    }
  } else {
    if (g_vibrationSensors[idx].endpoint != endpoint_) {
      g_vibrationSensors[idx].endpoint = endpoint_;
      changed = true;
    }
    g_vibrationSensors[idx].lastSeen = ZNP_GetCurrentTime();
    if (!g_vibrationSensors[idx].hasIeee) {
      g_vibrationSensors[idx].hasIeee = Device_GetDiscoveredIeee(shortAddr_, g_vibrationSensors[idx].ieee);
      if (g_vibrationSensors[idx].hasIeee)
        changed = true;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);

  if (changed)
    Device_Save();
  VibrationSensor_PostAssign(shortAddr_);
}

void VibrationSensor_UpdateIeee(uint16_t shortAddr_, const uint8_t *ieee_) {
  bool found = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      memcpy(g_vibrationSensors[i].ieee, ieee_, 8);
      g_vibrationSensors[i].hasIeee = true;
      found = true;
      break;
    }
  }
  if (found) {
    for (int i = g_numVibrationSensors - 1; i >= 0; i--) {
      if (g_vibrationSensors[i].shortAddr != shortAddr_ && g_vibrationSensors[i].hasIeee && memcmp(g_vibrationSensors[i].ieee, ieee_, 8) == 0) {
        for (int j = i; j < g_numVibrationSensors - 1; j++) {
          g_vibrationSensors[j] = g_vibrationSensors[j + 1];
        }
        g_numVibrationSensors--;
      }
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  if (found) {
    Device_Save();
    VibrationSensor_PostAssign(shortAddr_);
  }
}

void VibrationSensor_Setup(uint16_t shortAddr_) {
  pthread_mutex_lock(&g_deviceMutex);
  int idx = -1;
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      idx = i;
      break;
    }
  }
  if (idx == -1 || g_vibrationSensors[idx].configured) {
    pthread_mutex_unlock(&g_deviceMutex);
    return;
  }
  if (!g_vibrationSensors[idx].hasIeee) {
    g_vibrationSensors[idx].hasIeee = Device_GetDiscoveredIeee(shortAddr_, g_vibrationSensors[idx].ieee);
  }
  if (!g_vibrationSensors[idx].hasIeee) {
    pthread_mutex_unlock(&g_deviceMutex);
    printf("Vibration sensor 0x%04X missing IEEE - requesting...\n", shortAddr_);
    uint8_t reqPay[4] = {shortAddr_ & 0xFF, (shortAddr_ >> 8) & 0xFF, 0x01, 0x00};
    ZNP_Sreq(0x25, 0x01, reqPay, 4, NULL, 3000);
    return;
  }

  uint8_t sensorIeee[8];
  uint8_t endpoint = g_vibrationSensors[idx].endpoint;
  memcpy(sensorIeee, g_vibrationSensors[idx].ieee, 8);
  g_vibrationSensors[idx].configured = true;
  pthread_mutex_unlock(&g_deviceMutex);

  printf("Configuring Vibration Sensor 0x%04X...\n", shortAddr_);
  ZNP_ZdoBindReq(shortAddr_, sensorIeee, endpoint, 0x0500, g_coordinatorIeee, 8);
  usleep(500000);
  ZNP_WriteCieAddress(shortAddr_, endpoint, 0x20);
  usleep(500000);
  ZNP_SendZoneEnrollResponse(shortAddr_, endpoint, 0x21, 0x01);
  usleep(200000);
  printf("Configuration sent to Vibration Sensor 0x%04X!\n", shortAddr_);
}

void VibrationSensor_HandleEnroll(uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_) {
  printf("   -> Zone Enroll Request from Vibration Sensor 0x%04X, zone_type=0x%04X\n", shortAddr_, zoneType_);
  uint8_t zoneId = g_nextZoneId++;

  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      g_vibrationSensors[i].zoneId = zoneId;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  Device_Save();
  ZNP_SendZoneEnrollResponse(shortAddr_, endpoint_, transSeq_, zoneId);
}

void VibrationSensor_HandleStatus(uint16_t shortAddr_, uint16_t zoneStatus_, uint8_t zoneId_) {
  printf("   -> Zone Status Change from Vibration Sensor 0x%04X: zone_status=0x%04X, zone_id=%d\n", shortAddr_, zoneStatus_, zoneId_);

  pthread_mutex_lock(&g_deviceMutex);
  int idx = -1;
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      idx = i;
      break;
    }
  }
  
  if (idx != -1) {
    g_vibrationSensors[idx].lastZoneStatus = zoneStatus_;

    bool movement_active = ((zoneStatus_ & 0x0001) != 0); // Alarm 1
    bool vibration_active = ((zoneStatus_ & 0x0002) != 0); // Alarm 2

    // Handle Movement (Alarm 1)
    if (movement_active) {
      if (!g_vibrationSensors[idx].isMoving) {
        g_vibrationSensors[idx].isMoving = true;
        UseCase_Post(UC_MOVEMENT_DETECTED, shortAddr_, zoneStatus_);
      }
      g_vibrationSensors[idx].lastMovementTime = ZNP_GetCurrentTime();
    }

    // Handle Vibration (Alarm 2)
    if (vibration_active) {
      if (!g_vibrationSensors[idx].isVibrating) {
        g_vibrationSensors[idx].isVibrating = true;
        UseCase_Post(UC_VIBRATION_DETECTED, shortAddr_, zoneStatus_);
      }
      g_vibrationSensors[idx].lastVibrationTime = ZNP_GetCurrentTime();
    }
    // If all alarm bits are cleared, the sensor explicitly cleared the state
    else {
      // Do nothing! Let the 5-second polling loop clear the vibration state to enforce a minimum hold time.
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
}

void VibrationSensor_PrintStatus(void) {
  pthread_mutex_lock(&g_deviceMutex);
  printf("Registered Vibration Sensors (%d):\n", g_numVibrationSensors);
  double now = ZNP_GetCurrentTime();
  for (int i = 0; i < g_numVibrationSensors; i++) {
    printf("  - 0x%04X: IEEE=", g_vibrationSensors[i].shortAddr);
    if (g_vibrationSensors[i].hasIeee) {
      for (int j = 7; j >= 0; j--) {
        printf("%02x", g_vibrationSensors[i].ieee[j]);
      }
    } else {
      printf("Unknown");
    }
    printf(", ep=0x%02X, zone_id=%d, last_seen=%.1fs ago\n", g_vibrationSensors[i].endpoint, g_vibrationSensors[i].zoneId, now - g_vibrationSensors[i].lastSeen);
  }
  pthread_mutex_unlock(&g_deviceMutex);
}

bool VibrationSensor_IsKnown(uint16_t shortAddr_) {
  bool known = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      known = true;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  return known;
}

void VibrationSensor_UpdateSeen(uint16_t shortAddr_) {
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].shortAddr == shortAddr_) {
      g_vibrationSensors[i].lastSeen = ZNP_GetCurrentTime();
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
}

void VibrationSensor_DiscoverAllActiveEp(void) {
  pthread_mutex_lock(&g_deviceMutex);
  int tempNum = g_numVibrationSensors;
  uint16_t tempAddrs[MAX_VIBRATION_SENSORS];
  for (int i = 0; i < g_numVibrationSensors; i++) {
    tempAddrs[i] = g_vibrationSensors[i].shortAddr;
  }
  pthread_mutex_unlock(&g_deviceMutex);

  for (int i = 0; i < tempNum; i++) {
    ZNP_ZdoActiveEpReq(tempAddrs[i]);
    ZNP_QuerySimpleDesc(tempAddrs[i], 1);
  }
}

void VibrationSensor_PollAll(void) {
  double now = ZNP_GetCurrentTime();
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numVibrationSensors; i++) {
    // Check Vibration timeout
    if (g_vibrationSensors[i].isVibrating && (now - g_vibrationSensors[i].lastVibrationTime > 5.0)) {
      g_vibrationSensors[i].isVibrating = false;
      // Clear the Alarm 2 bit in lastZoneStatus
      g_vibrationSensors[i].lastZoneStatus &= ~0x0002;
      UseCase_Post(UC_VIBRATION_CLEARED, g_vibrationSensors[i].shortAddr, 0);
      printf("   -> Auto-cleared Vibration for Sensor 0x%04X (Timeout)\n", g_vibrationSensors[i].shortAddr);
    }

    // Check Movement timeout
    if (g_vibrationSensors[i].isMoving && (now - g_vibrationSensors[i].lastMovementTime > 5.0)) {
      g_vibrationSensors[i].isMoving = false;
      // Clear the Alarm 1 bit in lastZoneStatus
      g_vibrationSensors[i].lastZoneStatus &= ~0x0001;
      UseCase_Post(UC_MOVEMENT_CLEARED, g_vibrationSensors[i].shortAddr, 0);
      printf("   -> Auto-cleared Movement for Sensor 0x%04X (Timeout)\n", g_vibrationSensors[i].shortAddr);
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
}
#endif
