///
/// @file   main.c
/// @brief  Startup sequence, the incoming-frame dispatcher, device persistence,
///         and the interactive CLI for the ZNP host controller.
///
/// main() brings the coordinator up, then the main loop acts as the dispatcher:
/// it drains the AREQ event queue, classifies devices from ZDO responses, and
/// routes AF messages to the owning sensor's worker thread. Sensor device I/O
/// and the siren policy run on their own threads.
///
#include "config.h"
#include "znp_host.h"

#if ENABLE_SIREN
#include "siren.h"
#endif
#if ENABLE_AQARA_BUTTON
#include "aqara_button.h"
#endif
#if ENABLE_ONICS_BUTTON
#include "onics_button.h"
#endif
#if ENABLE_AQARA_OCCUPANCY
#include "aqara_occupancy.h"
#endif
#if ENABLE_CONTACT_SENSOR
#include "contact_sensor.h"
#endif
#if ENABLE_VIBRATION_SENSOR
#include "vibration_sensor.h"
#endif
#if ENABLE_OKOS_SIREN
#include "okos_siren.h"
#endif
#if ENABLE_AQARA_TVOC
#include "aqara_tvoc.h"
#endif

#include "cli.h"
#include "sensor_common.h"
#include "usecase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <pthread.h>

// Shared mutex protecting the global registries
pthread_mutex_t g_deviceMutex;

// Device registry persistence file. Relative to the current working directory
// (run the program from its own folder).
static const char DEVICES_FILE[] = "devices.txt";

// Offline detection thresholds.
// Only for ACTIVELY POLLED devices — battery sensors that are event-driven
// are NEVER flagged offline just because they are quiet.
// Occupancy: if we polled it but got NO reply within this window -> not
// responding.
#define OCC_POLL_NO_REPLY_THRESHOLD 90.0 ///< 90s: occupancy polled but silent

// Local (static) function declarations
static void
Main_HandleIncomingFrame(const MT_FRAME_T *frame_); ///< Route one AREQ frame.
static void Main_CheckDeviceOfflineStatus(
    double now_); ///< Log offline/not-responding devices.

void Device_AddDiscoveredIeee(uint16_t shortAddr_, const uint8_t *ieee_) {
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numDiscoveredIeees; i++) {
    if (g_discoveredIeees[i].shortAddr == shortAddr_) {
      memcpy(g_discoveredIeees[i].ieee, ieee_, 8);
      pthread_mutex_unlock(&g_deviceMutex);
      return;
    }
  }
  if (g_numDiscoveredIeees < MAX_DISCOVERED_IEEES) {
    g_discoveredIeees[g_numDiscoveredIeees].shortAddr = shortAddr_;
    memcpy(g_discoveredIeees[g_numDiscoveredIeees].ieee, ieee_, 8);
    g_numDiscoveredIeees++;
  }
  pthread_mutex_unlock(&g_deviceMutex);
}

bool Device_GetDiscoveredIeee(uint16_t shortAddr_, uint8_t *ieeeOut_) {
  bool found = false;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numDiscoveredIeees; i++) {
    if (g_discoveredIeees[i].shortAddr == shortAddr_) {
      if (ieeeOut_ != NULL) {
        memcpy(ieeeOut_, g_discoveredIeees[i].ieee, 8);
      }
      found = true;
      break;
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
  return found;
}

#define MAX_PENDING_DISCOVERIES 32
typedef struct {
  uint16_t shortAddr;
  double lastQueryTime;
} PENDING_DISCOVERY_T;

static PENDING_DISCOVERY_T s_pendingDiscoveries[MAX_PENDING_DISCOVERIES];
static int s_numPendingDiscoveries = 0;

///
/// @brief  Rate-limit descriptor queries for an unknown device to once per 30
/// s.
///
/// Prevents a chatty unknown device from triggering a flood of Simple_Desc /
/// discovery requests while its first query is still in flight.
///
/// @param  shortAddr_  Device network address.
/// @return true if the caller should issue a (re)query now; false if throttled.
///
static bool Device_ShouldQuery(uint16_t shortAddr_) {
  double now = ZNP_GetCurrentTime();
  bool result = true;
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < s_numPendingDiscoveries; i++) {
    if (s_pendingDiscoveries[i].shortAddr == shortAddr_) {
      if (now - s_pendingDiscoveries[i].lastQueryTime < 30.0) {
        result = false;
      } else {
        s_pendingDiscoveries[i].lastQueryTime = now;
      }
      pthread_mutex_unlock(&g_deviceMutex);
      return result;
    }
  }

  if (s_numPendingDiscoveries < MAX_PENDING_DISCOVERIES) {
    s_pendingDiscoveries[s_numPendingDiscoveries].shortAddr = shortAddr_;
    s_pendingDiscoveries[s_numPendingDiscoveries].lastQueryTime = now;
    s_numPendingDiscoveries++;
  }
  pthread_mutex_unlock(&g_deviceMutex);
  return true;
}

void Device_Save(void) {
  pthread_mutex_lock(&g_deviceMutex);
  FILE *file = fopen(DEVICES_FILE, "w");
  if (!file) {
    pthread_mutex_unlock(&g_deviceMutex);
    return;
  }

#if ENABLE_SIREN
  for (int i = 0; i < g_numSirens; i++) {
    if (g_sirens[i].hasIeee) {
      fprintf(file, "siren %04X %02X ", g_sirens[i].shortAddr,
              g_sirens[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_sirens[i].ieee[j]);
      }
      fprintf(file, " %d %d %u %u\n", g_sirens[i].hasIeee ? 1 : 0,
              g_sirens[i].zoneId, g_sirens[i].volume, g_sirens[i].mode);
    }
  }
#endif
#if ENABLE_OKOS_SIREN
  for (int i = 0; i < g_numOkosSirens; i++) {
    if (g_okosSirens[i].hasIeee) {
      fprintf(file, "okos_siren %04X %02X ", g_okosSirens[i].shortAddr,
              g_okosSirens[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_okosSirens[i].ieee[j]);
      }
      fprintf(file, " %d %d %u %u %u\n", g_okosSirens[i].hasIeee ? 1 : 0,
              g_okosSirens[i].zoneId, g_okosSirens[i].volume,
              g_okosSirens[i].toneId, g_okosSirens[i].strobeMode);
    }
  }
#endif

#if ENABLE_AQARA_BUTTON
  for (int i = 0; i < g_numAqaraButtons; i++) {
    if (g_aqaraButtons[i].hasIeee) {
      fprintf(file, "aqara %04X %02X ", g_aqaraButtons[i].shortAddr,
              g_aqaraButtons[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_aqaraButtons[i].ieee[j]);
      }
      fprintf(file, " %d\n", g_aqaraButtons[i].hasIeee ? 1 : 0);
    }
  }
#endif

#if ENABLE_ONICS_BUTTON
  for (int i = 0; i < g_numOnicsButtons; i++) {
    if (g_onicsButtons[i].hasIeee) {
      fprintf(file, "onics %04X %02X ", g_onicsButtons[i].shortAddr,
              g_onicsButtons[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_onicsButtons[i].ieee[j]);
      }
      fprintf(file, " %d %d\n", g_onicsButtons[i].hasIeee ? 1 : 0,
              g_onicsButtons[i].zoneId);
    }
  }
#endif

#if ENABLE_AQARA_OCCUPANCY
  for (int i = 0; i < g_numAqaraOccupancies; i++) {
    if (g_aqaraOccupancies[i].hasIeee) {
      fprintf(file, "occupancy %04X %02X ", g_aqaraOccupancies[i].shortAddr,
              g_aqaraOccupancies[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_aqaraOccupancies[i].ieee[j]);
      }
      fprintf(file, " %d", g_aqaraOccupancies[i].hasIeee ? 1 : 0);
      for (int z = 0; z < MAX_OCCUPANCY_ZONES; z++) {
        fprintf(file, " %d %u %u",
                g_aqaraOccupancies[i].zones[z].isActive ? 1 : 0,
                g_aqaraOccupancies[i].zones[z].minCm,
                g_aqaraOccupancies[i].zones[z].maxCm);
      }
      fprintf(file, " %u\n", g_aqaraOccupancies[i].lightThreshold);
    }
  }
#endif

#if ENABLE_CONTACT_SENSOR
  for (int i = 0; i < g_numContactSensors; i++) {
    if (g_contactSensors[i].hasIeee) {
      fprintf(file, "contact %04X %02X ", g_contactSensors[i].shortAddr,
              g_contactSensors[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_contactSensors[i].ieee[j]);
      }
      fprintf(file, " %d %d\n", g_contactSensors[i].hasIeee ? 1 : 0,
              g_contactSensors[i].zoneId);
    }
  }
#endif

#if ENABLE_VIBRATION_SENSOR
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (g_vibrationSensors[i].hasIeee) {
      fprintf(file, "vibration %04X %02X ", g_vibrationSensors[i].shortAddr,
              g_vibrationSensors[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_vibrationSensors[i].ieee[j]);
      }
      fprintf(file, " %d %d %d\n", g_vibrationSensors[i].hasIeee ? 1 : 0,
              g_vibrationSensors[i].zoneId, g_vibrationSensors[i].sensitivity);
    }
  }
#endif
#if ENABLE_AQARA_TVOC
  for (int i = 0; i < g_numAqaraTvocs; i++) {
    if (g_aqaraTvocs[i].hasIeee) {
      fprintf(file, "tvoc %04X %02X ", g_aqaraTvocs[i].shortAddr,
              g_aqaraTvocs[i].endpoint);
      for (int j = 0; j < 8; j++) {
        fprintf(file, "%02X", g_aqaraTvocs[i].ieee[j]);
      }
      fprintf(file, " %d\n", g_aqaraTvocs[i].hasIeee ? 1 : 0);
    }
  }
#endif

  fclose(file);
  pthread_mutex_unlock(&g_deviceMutex);
}

void Device_Load(void) {
  pthread_mutex_lock(&g_deviceMutex);
  FILE *file = fopen(DEVICES_FILE, "r");
  if (!file) {
    pthread_mutex_unlock(&g_deviceMutex);
    return;
  }

  char type[32];
  unsigned int shortAddr;
  unsigned int endpoint;
  unsigned int hasIeee;
  unsigned int zoneId;
  char ieeeStr[32];

  while (fscanf(file, "%31s %X %X %31s", type, &shortAddr, &endpoint,
                ieeeStr) == 4) {
    uint8_t ieee[8];
    for (int i = 0; i < 8; i++) {
      unsigned int byteVal;
      sscanf(&ieeeStr[i * 2], "%2X", &byteVal);
      ieee[i] = (uint8_t)byteVal;
    }

    if (0) {
    }
#if ENABLE_SIREN
    else if (strcmp(type, "siren") == 0) {
      uint32_t vol = 2;
      uint32_t mod = 1;
      int items = fscanf(file, "%u %u %u %u", &hasIeee, &zoneId, &vol, &mod);
      if (items == 1) {
        zoneId = 0;
      }
      if (items >= 1 && g_numSirens < MAX_SIRENS) {
        g_sirens[g_numSirens].shortAddr = shortAddr;
        g_sirens[g_numSirens].endpoint = endpoint;
        g_sirens[g_numSirens].lastSeen = ZNP_GetCurrentTime();
        g_sirens[g_numSirens].hasIeee = (hasIeee != 0);
        memcpy(g_sirens[g_numSirens].ieee, ieee, 8);
        g_sirens[g_numSirens].zoneId = zoneId;
        g_sirens[g_numSirens].volume = vol;
        g_sirens[g_numSirens].mode = mod;
        g_sirens[g_numSirens].configured = true;
        g_numSirens++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_AQARA_BUTTON
    else if (strcmp(type, "aqara") == 0) {
      if (fscanf(file, "%u", &hasIeee) == 1 &&
          g_numAqaraButtons < MAX_AQARA_BUTTONS) {
        g_aqaraButtons[g_numAqaraButtons].shortAddr = shortAddr;
        g_aqaraButtons[g_numAqaraButtons].endpoint = endpoint;
        g_aqaraButtons[g_numAqaraButtons].lastSeen = ZNP_GetCurrentTime();
        g_aqaraButtons[g_numAqaraButtons].hasIeee = (hasIeee != 0);
        memcpy(g_aqaraButtons[g_numAqaraButtons].ieee, ieee, 8);
        g_aqaraButtons[g_numAqaraButtons].configured = true;
        g_numAqaraButtons++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_ONICS_BUTTON
    else if (strcmp(type, "onics") == 0) {
      if (fscanf(file, "%u %u", &hasIeee, &zoneId) == 2 &&
          g_numOnicsButtons < MAX_ONICS_BUTTONS) {
        g_onicsButtons[g_numOnicsButtons].shortAddr = shortAddr;
        g_onicsButtons[g_numOnicsButtons].endpoint = endpoint;
        g_onicsButtons[g_numOnicsButtons].lastSeen = ZNP_GetCurrentTime();
        g_onicsButtons[g_numOnicsButtons].hasIeee = (hasIeee != 0);
        memcpy(g_onicsButtons[g_numOnicsButtons].ieee, ieee, 8);
        g_onicsButtons[g_numOnicsButtons].zoneId = zoneId;
        g_onicsButtons[g_numOnicsButtons].configured = true;
        g_numOnicsButtons++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_AQARA_OCCUPANCY
    else if (strcmp(type, "occupancy") == 0) {
      // Default values in case fscanf fails to read them (legacy format)
      int zActive[MAX_OCCUPANCY_ZONES] = {1, 0, 0, 0};
      uint32_t zMin[MAX_OCCUPANCY_ZONES] = {0, 0, 0, 0};
      uint32_t zMax[MAX_OCCUPANCY_ZONES] = {600, 0, 0, 0};

      int scanned = fscanf(file, "%u", &hasIeee);
      if (scanned >= 1 && g_numAqaraOccupancies < MAX_AQARA_OCCUPANCY) {
        // Try to read the zones array
        for (int z = 0; z < MAX_OCCUPANCY_ZONES; z++) {
          if (fscanf(file, "%d %u %u", &zActive[z], &zMin[z], &zMax[z]) != 3)
            break;
        }

        uint32_t lightThresh = 18000; // default
        if (fscanf(file, "%u", &lightThresh) != 1) {
          lightThresh = 18000;
        }

        g_aqaraOccupancies[g_numAqaraOccupancies].shortAddr = shortAddr;
        g_aqaraOccupancies[g_numAqaraOccupancies].endpoint = endpoint;
        g_aqaraOccupancies[g_numAqaraOccupancies].lastSeen =
            ZNP_GetCurrentTime();
        g_aqaraOccupancies[g_numAqaraOccupancies].hasIeee = (hasIeee != 0);
        memcpy(g_aqaraOccupancies[g_numAqaraOccupancies].ieee, ieee, 8);

        for (int z = 0; z < MAX_OCCUPANCY_ZONES; z++) {
          g_aqaraOccupancies[g_numAqaraOccupancies].zones[z].isActive =
              (zActive[z] != 0);
          g_aqaraOccupancies[g_numAqaraOccupancies].zones[z].minCm = zMin[z];
          g_aqaraOccupancies[g_numAqaraOccupancies].zones[z].maxCm = zMax[z];
        }

        g_aqaraOccupancies[g_numAqaraOccupancies].lightThreshold =
            (uint16_t)lightThresh;
        g_aqaraOccupancies[g_numAqaraOccupancies].configured = true;
        g_numAqaraOccupancies++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_CONTACT_SENSOR
    else if (strcmp(type, "contact") == 0) {
      if (fscanf(file, "%u %u", &hasIeee, &zoneId) == 2 &&
          g_numContactSensors < MAX_CONTACT_SENSORS) {
        g_contactSensors[g_numContactSensors].shortAddr = shortAddr;
        g_contactSensors[g_numContactSensors].endpoint = endpoint;
        g_contactSensors[g_numContactSensors].lastSeen = ZNP_GetCurrentTime();
        g_contactSensors[g_numContactSensors].hasIeee = (hasIeee != 0);
        memcpy(g_contactSensors[g_numContactSensors].ieee, ieee, 8);
        g_contactSensors[g_numContactSensors].zoneId = zoneId;
        g_contactSensors[g_numContactSensors].configured = true;
        g_numContactSensors++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_VIBRATION_SENSOR
    else if (strcmp(type, "vibration") == 0) {
      unsigned int sens = 10;
      int scanned = fscanf(file, "%u %u %u", &hasIeee, &zoneId, &sens);
      if (scanned >= 2 && g_numVibrationSensors < MAX_VIBRATION_SENSORS) {
        g_vibrationSensors[g_numVibrationSensors].shortAddr = shortAddr;
        g_vibrationSensors[g_numVibrationSensors].endpoint = endpoint;
        g_vibrationSensors[g_numVibrationSensors].lastSeen =
            ZNP_GetCurrentTime();
        g_vibrationSensors[g_numVibrationSensors].hasIeee = (hasIeee != 0);
        memcpy(g_vibrationSensors[g_numVibrationSensors].ieee, ieee, 8);
        g_vibrationSensors[g_numVibrationSensors].zoneId = zoneId;
        g_vibrationSensors[g_numVibrationSensors].sensitivity = sens;
        g_vibrationSensors[g_numVibrationSensors].configured = true;
        g_vibrationSensors[g_numVibrationSensors].isVibrating = false;
        g_vibrationSensors[g_numVibrationSensors].lastVibrationTime = 0.0;
        g_vibrationSensors[g_numVibrationSensors].isMoving = false;
        g_vibrationSensors[g_numVibrationSensors].lastMovementTime = 0.0;
        g_numVibrationSensors++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_OKOS_SIREN
    else if (strcmp(type, "okos_siren") == 0) {
      uint32_t vol = 2, toneId = 1, strobe = 0;
      int scanned = fscanf(file, "%u %u %u %u %u", &hasIeee, &zoneId, &vol,
                           &toneId, &strobe);
      if (scanned >= 2 && g_numOkosSirens < MAX_OKOS_SIRENS) {
        OKOS_SIREN_T *s = &g_okosSirens[g_numOkosSirens];
        s->shortAddr = shortAddr;
        s->endpoint = (uint8_t)endpoint;
        s->lastSeen = ZNP_GetCurrentTime();
        s->hasIeee = (hasIeee != 0);
        memcpy(s->ieee, ieee, 8);
        s->zoneId = (uint8_t)zoneId;
        s->volume = (uint8_t)vol;
        s->toneId = (uint8_t)toneId;
        s->strobeMode = (uint8_t)strobe;
        s->configured = true;
        s->temperatureCdeg = 0x7FFF;
        s->humidityHpct = 0xFFFF;
        s->batteryPct = 0xFF;
        g_numOkosSirens++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
#if ENABLE_AQARA_TVOC
    else if (strcmp(type, "tvoc") == 0) {
      if (fscanf(file, "%u", &hasIeee) == 1 &&
          g_numAqaraTvocs < MAX_AQARA_TVOC) {
        g_aqaraTvocs[g_numAqaraTvocs].shortAddr = shortAddr;
        g_aqaraTvocs[g_numAqaraTvocs].endpoint = endpoint;
        g_aqaraTvocs[g_numAqaraTvocs].lastSeen = ZNP_GetCurrentTime();
        g_aqaraTvocs[g_numAqaraTvocs].hasIeee = (hasIeee != 0);
        memcpy(g_aqaraTvocs[g_numAqaraTvocs].ieee, ieee, 8);
        g_numAqaraTvocs++;
        Device_AddDiscoveredIeee(shortAddr, ieee);
      }
    }
#endif
  }

  fclose(file);
  LOG_INFO(" Loaded existing devices from devices.txt\n");
  pthread_mutex_unlock(&g_deviceMutex);
}

static void Main_LoadInstallCodes(void) {
  LOG_INFO("[5.8] Loading Install Codes from install_codes.txt...\n");
  FILE *file = fopen("install_codes.txt", "r");
  if (!file) {
    LOG_WARNING("   Could not open install_codes.txt, skipping.\n");
    return;
  }
  char line[256];
  int count = 0;
  while (fgets(line, sizeof(line), file)) {
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') continue;
    char ieeeStr[32];
    char icStr[64];
    if (sscanf(line, "%31s %63s", ieeeStr, icStr) == 2) {
      if (strlen(ieeeStr) == 16 && strlen(icStr) == 36) {
        uint8_t ieee[8];
        uint8_t ic[18];
        for (int i = 0; i < 8; i++) {
          unsigned int byteVal;
          sscanf(&ieeeStr[i * 2], "%2X", &byteVal);
          ieee[i] = (uint8_t)byteVal;
        }
        for (int i = 0; i < 18; i++) {
          unsigned int byteVal;
          sscanf(&icStr[i * 2], "%2X", &byteVal);
          ic[i] = (uint8_t)byteVal;
        }
        if (ZNP_BdbAddInstallCode(ieee, ic)) {
          LOG_INFO("   Added IC for IEEE: %s\n", ieeeStr);
          count++;
        } else {
          LOG_WARNING("   Failed to add IC for IEEE: %s\n", ieeeStr);
        }
      } else {
        LOG_WARNING("   Invalid format in install_codes.txt: %s", line);
      }
    }
  }
  fclose(file);
  if (count == 0) {
    LOG_INFO("   No valid install codes loaded.\n");
  }
}

///
/// @brief  Program entry point: parse args, bring up the coordinator, run loop.
///
/// Initializes modules, loads persisted devices, opens the serial port, runs
/// the ZNP startup sequence (ping, optional factory-new, network start,
/// endpoint + callback registration, permit join, discovery), starts the
/// worker threads, then enters the dispatcher loop.
///
/// @param  argc  Argument count.
/// @param  argv  Arguments: optional serial port path and/or -f/--factory-new.
/// @return 0 on normal exit; 1 on a fatal startup error.
///
int main(int argc, char *argv[]) {
  setvbuf(stdin, NULL, _IOLBF, 0);
  setvbuf(stdout, NULL, _IOLBF, 0);

  // Initialize g_deviceMutex as a recursive mutex
  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
  pthread_mutex_init(&g_deviceMutex, &attr);
  pthread_mutexattr_destroy(&attr);

  // Initialize the use-case layer and each sensor module (lists + inboxes).
  UseCase_Init();
#if ENABLE_SIREN
  Siren_Init();
#endif
#if ENABLE_AQARA_BUTTON
  AqaraButton_Init();
#endif
#if ENABLE_ONICS_BUTTON
  OnicsButton_Init();
#endif
#if ENABLE_AQARA_OCCUPANCY
  AqaraOccupancy_Init();
#endif
#if ENABLE_CONTACT_SENSOR
  ContactSensor_Init();
#endif
#if ENABLE_VIBRATION_SENSOR
  VibrationSensor_Init();
#endif
#if ENABLE_OKOS_SIREN
  OkosSiren_Init();
#endif
#if ENABLE_AQARA_TVOC
  AqaraTvoc_Init();
#endif

  // Load persisted devices
  Device_Load();

  const char *port = PORT_DEFAULT;
  bool forceFactoryNew = false;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-f") == 0 || strcmp(argv[i], "--factory-new") == 0) {
      forceFactoryNew = true;
    } else if (argv[i][0] != '-') {
      port = argv[i];
    }
  }

  printf("\n============================================================\n");
  printf("  ZNP MT Host Controller for Zigbee Sensor\n");
  printf("  Firmware Version: v%s\n", APP_VERSION);
  printf("  Port: %s  (Factory New: %s)\n", port,
         forceFactoryNew ? "YES" : "NO");
  printf("============================================================\n\n");

  if (!ZNP_Init(port)) {
    LOG_ERROR("Failed to open serial port %s\n", port);
    return 1;
  }

  sleep(1);

  // 1. Ping coordinator
  LOG_INFO("[1] SYS_PING...\n");
  if (!ZNP_SysPing()) {
    LOG_DEBUG("No response from ZNP. Check port and connections.\n");
    ZNP_Close();
    return 1;
  }
  LOG_INFO("   ZNP responsive\n\n");

  // 2. Get device info
  LOG_INFO("[2] UTIL_GET_DEVICE_INFO...\n");
  int state = ZNP_UtilGetDeviceInfo();
  LOG_RAW("\n");

  if (state == 9 && !forceFactoryNew) {
    LOG_DEBUG("Coordinator already active — skipping startup.\n\n");
  } else {
    if (forceFactoryNew) {
      LOG_INFO("[3] Factory-new sequence (clear NV + write config)...\n");
      ZNP_FactoryNew();
      LOG_RAW("\n");
    } else {
      LOG_INFO("[3] SYS_RESET_REQ (soft reset)...\n");
      ZNP_SysResetReq(false);
      sleep(1);
      LOG_RAW("\n");
    }

    LOG_INFO("[4] ZDO_STARTUP_FROM_APP...\n");
    ZNP_ZdoStartupFromApp(100);
    LOG_RAW("\n");

    LOG_INFO("[5] Waiting for coordinator state (state=9)...\n");
    bool active = false;
    double deadline = ZNP_GetCurrentTime() + 30.0;
    MT_FRAME_T rx;
    while (ZNP_GetCurrentTime() < deadline) {
      if (EventQueue_Pop(&g_eventQueue, &rx, 100)) {
        // ZDO_STATE_CHANGE_IND: cmd0=0x45, cmd1=0xC0
        if (rx.cmd0 == 0x45 && rx.cmd1 == 0xC0 && rx.len >= 1) {
          uint8_t devState = rx.payload[0];
          LOG_DEBUG("-> State: %d\n", devState);
          if (devState == 9) {
            LOG_INFO("   COORDINATOR ACTIVE!\n");
            active = true;
            break;
          }
        } else {
          // Let Main_HandleIncomingFrame process other events (joins etc.)
          Main_HandleIncomingFrame(&rx);
        }
      }
    }
    LOG_RAW("\n");

    if (!active) {
      LOG_DEBUG("[5b] Re-checking device info...\n");
      state = ZNP_UtilGetDeviceInfo();
      LOG_RAW("\n");
      if (state != 9) {
        LOG_DEBUG("Coordinator did not start.\n");
        ZNP_Close();
        return 1;
      }
    }
  }

  // 5.5 Register endpoint and ZDO callbacks.
  // Input clusters must include everything our devices REPORT on, or the ZNP
  // won't route those reports to us (and devices won't bind for reporting to a
  // cluster the coordinator doesn't advertise). 0xFCC0 = Aqara manufacturer
  // cluster (occupancy/presence attr 0x0142), 0x0012 = Multistate Input.
  LOG_INFO("[5.5] Registering Application Endpoint and ZDO Callbacks...\n");
  uint16_t inClusters[8] = {0x0000, 0x0003, 0x0004, 0x0005,
                            0x0006, 0x0406, 0xFCC0, 0x0012};
  uint16_t outClusters[7] = {0x0500, 0x0502, 0x0406, 0xFCC0,
                             0x0402, 0x0405, 0x0400};
  ZNP_AfRegister(8, 0x0104, 0x0007, 1, 0, 8, inClusters, 7, outClusters);

  ZNP_ZdoMsgCbRegister(0x8001); // IEEE_addr_rsp
  ZNP_ZdoMsgCbRegister(0x8004); // Simple_desc_rsp
  ZNP_ZdoMsgCbRegister(0x8005); // Active_EP_rsp
  ZNP_ZdoMsgCbRegister(0x8006); // Match_desc_rsp
  ZNP_ZdoMsgCbRegister(0x0013); // Device_annce
  LOG_RAW("\n");

  // Turn Red LED OFF initially

  // 5.7 Relax the Trust Center key-exchange policy. Zigbee 3.0 defaults to
  // kicking any joiner that does not upgrade its TC link key within a few
  // seconds; many Aqara sensors never do, so they rejoin endlessly. Allowing
  // them to stay on the global key lets them settle on the network.
  LOG_INFO("[5.7] Relaxing Trust Center key-exchange policy...\n");
  ZNP_BdbSetTcRequireKeyExchange(false);
  LOG_RAW("\n");

  // Load Install Codes from file
  Main_LoadInstallCodes();
  LOG_RAW("\n");

  // 6. Open permit join
  LOG_INFO("[6] Enforcing strict Install Code security policy...\n");
  if (ZNP_BdbSetJoinUsesInstallCodeKey(false)) {
    LOG_INFO("   SUCCESS! Only devices with a registered Install Code can now "
             "join.\n");
  } else {
    LOG_WARNING("   Failed to set JoinUsesInstallCodeKey policy!\n");
  }
  LOG_RAW("\n");

  // 7. Open permit join
  LOG_INFO("[7] Opening permit join (all methods)....\n");
  ZNP_PermitJoin(PERMIT_JOIN_DURATION);
  LOG_RAW("\n");

  // 7. Discover existing devices
  LOG_INFO("[7] Discovering existing devices in the network...\n");
  uint16_t sirenIn = 0x0502;
  ZNP_ZdoMatchDescReq(0xFFFD, 0x0104, 1, &sirenIn, 0, NULL);
  uint16_t btnIn = 0x0500;
  ZNP_ZdoMatchDescReq(0xFFFD, 0x0104, 1, &btnIn, 0, NULL);
  uint16_t btnOut = 0x0006;
  ZNP_ZdoMatchDescReq(0xFFFD, 0x0104, 0, NULL, 1, &btnOut);
  uint16_t occIn = 0x0406;
  ZNP_ZdoMatchDescReq(0xFFFD, 0x0104, 1, &occIn, 0, NULL);
  LOG_RAW("\n");

  // Start the use-case thread and one worker thread per sensor. From here on
  // the main loop only routes frames; all sensor device I/O and the siren
  // policy run on their own threads.
  UseCase_Start();
#if ENABLE_SIREN
  Siren_Start();
#endif
#if ENABLE_AQARA_BUTTON
  AqaraButton_Start();
#endif
#if ENABLE_ONICS_BUTTON
  OnicsButton_Start();
#endif
#if ENABLE_AQARA_OCCUPANCY
  AqaraOccupancy_Start();
#endif
#if ENABLE_CONTACT_SENSOR
  ContactSensor_Start();
#endif
#if ENABLE_VIBRATION_SENSOR
  VibrationSensor_Start();
#endif
#if ENABLE_OKOS_SIREN
  OkosSiren_Start();
#endif
#if ENABLE_AQARA_TVOC
  AqaraTvoc_Start();
#endif

  // Start CLI thread
  Cli_Start();

  LOG_INFO(" Coordinator is active. Permit join is OPEN (%ds).\n",
           PERMIT_JOIN_DURATION);
  LOG_INFO("   Enter CLI commands (type 'help' for info). Press Ctrl+C to "
           "exit.\n\n");

  // double lastRefresh = ZNP_GetCurrentTime(); // Unused since auto-refresh is
  // disabled
  MT_FRAME_T eventFrame;

  while (1) {
    // Pop events from the event queue (10 ms wait max)
    if (EventQueue_Pop(&g_eventQueue, &eventFrame, 10)) {
      Main_HandleIncomingFrame(&eventFrame);
    }

    // Auto-refresh permit join if needed (DISABLED PER USER REQUEST)
    double now = ZNP_GetCurrentTime();
    /*
    if ( now - lastRefresh > PERMIT_JOIN_REFRESH )
    {
        LOG_DEBUG("[Auto-Refresh] Re-opening permit join...\n" );
        ZNP_PermitJoin( PERMIT_JOIN_DURATION );
        lastRefresh = now;
    }
    */

    // Offline watchdog: scan all devices every 60s
    static double lastOfflineCheck = 0;
    if (now - lastOfflineCheck >= 60.0) {
      lastOfflineCheck = now;
      Main_CheckDeviceOfflineStatus(now);
    }

#if ENABLE_VIBRATION_SENSOR
    VibrationSensor_PollAll();
#endif
#if ENABLE_SIREN
    Siren_PollAll();
#endif
#if ENABLE_OKOS_SIREN
    OkosSiren_PollAll();
#endif

    usleep(5000); // Small yield
  }

  ZNP_Close();
  return 0;
}

static uint16_t s_offlineDevices[256];
static int s_numOfflineDevices = 0;

bool Device_IsOffline(uint16_t addr_) {
  for (int i = 0; i < s_numOfflineDevices; i++) {
    if (s_offlineDevices[i] == addr_)
      return true;
  }
  return false;
}

const char *Device_GetName(uint16_t addr_) {
#if ENABLE_SIREN
  if (Siren_IsKnown(addr_))
    return "Siren";
#endif
#if ENABLE_AQARA_BUTTON
  if (AqaraButton_IsKnown(addr_))
    return "Aqara Button";
#endif
#if ENABLE_ONICS_BUTTON
  if (OnicsButton_IsKnown(addr_))
    return "Onics Button";
#endif
#if ENABLE_CONTACT_SENSOR
  if (ContactSensor_IsKnown(addr_))
    return "Contact Sensor";
#endif
#if ENABLE_VIBRATION_SENSOR
  if (VibrationSensor_IsKnown(addr_))
    return "Vibration Sensor";
#endif
#if ENABLE_AQARA_OCCUPANCY
  if (AqaraOccupancy_IsKnown(addr_))
    return "Presence Sensor";
#endif
#if ENABLE_OKOS_SIREN
  if (OkosSiren_IsKnown(addr_))
    return "Okos Smart Siren";
#endif
#if ENABLE_AQARA_TVOC
  if (AqaraTvoc_IsKnown(addr_))
    return "Aqara Air Quality Sensor";
#endif
  return "Unknown Device";
}

static void Main_SetOffline(uint16_t addr_) {
  if (!Device_IsOffline(addr_) && s_numOfflineDevices < 256) {
    s_offlineDevices[s_numOfflineDevices++] = addr_;
    UseCase_Post(UC_DEVICE_OFFLINE, addr_, 0, 0);
  }
}

static void Main_SetOnline(uint16_t addr_) {
  for (int i = 0; i < s_numOfflineDevices; i++) {
    if (s_offlineDevices[i] == addr_) {
      s_offlineDevices[i] = s_offlineDevices[--s_numOfflineDevices];
      UseCase_Post(UC_DEVICE_ONLINE, addr_, 0, 0);
      return;
    }
  }
}

///
/// @brief  Check if actively-polled devices are failing to respond to polls.
///
/// DESIGN INTENT:
///   - Battery-powered EVENT-DRIVEN sensors (buttons, contact, vibration) are
///     NEVER flagged here. They only transmit when something happens (a press,
///     door open, vibration). Silence is normal and expected for hours or days.
///     These can only be checked via an explicit CLI 'discover <addr>' command.
///
///   - The OCCUPANCY sensor is actively polled every ~3 seconds. If we have
///     sent polls but got NO reply for OCC_POLL_NO_REPLY_THRESHOLD seconds,
///     that means it was supposed to respond (we asked it) but didn't.
///     Only in that case do we log "NOT RESPONDING".
///
///   - The SIREN is mains-powered. When the siren watchdog thread tries to
///     send a warning and gets MAC_NO_ACK, that is logged immediately by the
///     siren module itself. No passive check needed here.
///
/// @param  now_  Current timestamp in seconds.
/// @return None.
///
static void Main_CheckDeviceOfflineStatus(double now_) {
#if ENABLE_AQARA_OCCUPANCY
  // Occupancy: compare lastPolled vs lastSeen.
  // If we polled it (lastPolled > 0) and the last reply is older than the
  // poll timestamp by more than OCC_POLL_NO_REPLY_THRESHOLD, the sensor
  // was asked to respond but stayed silent.
  pthread_mutex_lock(&g_deviceMutex);
  for (int i = 0; i < g_numAqaraOccupancies; i++) {
    double lastPolled = g_aqaraOccupancies[i].lastPolled;
    double lastSeen = g_aqaraOccupancies[i].lastSeen;
    if (lastPolled > 0 && lastPolled > lastSeen &&
        (now_ - lastPolled) > OCC_POLL_NO_REPLY_THRESHOLD) {
      LOG_DEBUG(
          "\U000026a0\ufe0f  [NOT RESPONDING] Occupancy Sensor 0x%04X: "
          "polled %.0fs ago but NO reply received! "
          "(last seen %.0fs ago) — radar may be frozen, check power/range.\n",
          g_aqaraOccupancies[i].shortAddr, now_ - lastPolled, now_ - lastSeen);
    }
  }
  pthread_mutex_unlock(&g_deviceMutex);
#else
  (void)now_;
#endif
  // Check sleepy end devices (Buttons, Contact, Vibration) for a 2-hour timeout
  // Check mains-powered routers (Sirens, Occupancy) for a 5-minute timeout
  pthread_mutex_lock(&g_deviceMutex);
#if ENABLE_AQARA_OCCUPANCY
  for (int i = 0; i < g_numAqaraOccupancies; i++) {
    if (now_ - g_aqaraOccupancies[i].lastSeen > 300.0)
      Main_SetOffline(g_aqaraOccupancies[i].shortAddr);
  }
#endif
#if ENABLE_SIREN
  for (int i = 0; i < g_numSirens; i++) {
    if (now_ - g_sirens[i].lastSeen > 300.0)
      Main_SetOffline(g_sirens[i].shortAddr);
  }
#endif
#if ENABLE_OKOS_SIREN
  for (int i = 0; i < g_numOkosSirens; i++) {
    // Okos Sirens have battery backup and may enter deep sleep, reporting only
    // every few hours.
    if (now_ - g_okosSirens[i].lastSeen > 7200.0)
      Main_SetOffline(g_okosSirens[i].shortAddr);
  }
#endif
#if ENABLE_CONTACT_SENSOR
  for (int i = 0; i < g_numContactSensors; i++) {
    if (now_ - g_contactSensors[i].lastSeen > 7200.0)
      Main_SetOffline(g_contactSensors[i].shortAddr);
  }
#endif
#if ENABLE_VIBRATION_SENSOR
  for (int i = 0; i < g_numVibrationSensors; i++) {
    if (now_ - g_vibrationSensors[i].lastSeen > 7200.0)
      Main_SetOffline(g_vibrationSensors[i].shortAddr);
  }
#endif
#if ENABLE_AQARA_TVOC
  for (int i = 0; i < g_numAqaraTvocs; i++) {
    // Battery-powered e-ink display sensor
    if (now_ - g_aqaraTvocs[i].lastSeen > 7200.0)
      Main_SetOffline(g_aqaraTvocs[i].shortAddr);
  }
#endif
#if ENABLE_AQARA_BUTTON
  for (int i = 0; i < g_numAqaraButtons; i++) {
    if (now_ - g_aqaraButtons[i].lastSeen > 7200.0)
      Main_SetOffline(g_aqaraButtons[i].shortAddr);
  }
#endif
#if ENABLE_ONICS_BUTTON
  for (int i = 0; i < g_numOnicsButtons; i++) {
    if (now_ - g_onicsButtons[i].lastSeen > 7200.0)
      Main_SetOffline(g_onicsButtons[i].shortAddr);
  }
#endif
  pthread_mutex_unlock(&g_deviceMutex);
}

// ---------------------------------------------------------------------------
// Incoming Frame Dispatcher
// ---------------------------------------------------------------------------
///
/// @brief  Classify and route one asynchronous ZNP frame.
///
/// Handles device announces (triggering discovery), ZDO responses (IEEE /
/// active-EP / simple-desc / match-desc, driving device classification), and AF
/// incoming messages (routed to the owning sensor's worker thread). Does no
/// sensor-specific parsing itself.
///
/// @param  frame_  The AREQ frame popped from ::g_eventQueue.
/// @return None.
///
static void Main_HandleIncomingFrame(const MT_FRAME_T *frame_) {
  // 1. ZDO Device Announce (cmd0: 0x45, cmd1: 0xC1)
  if (frame_->cmd0 == 0x45 && frame_->cmd1 == 0xC1) {
    if (frame_->len < 12) {
      return;
    }
    uint16_t nwkAddr = frame_->payload[2] | (frame_->payload[3] << 8);
    uint8_t ieeeBytes[8];
    memcpy(ieeeBytes, &frame_->payload[4], 8);

    const char *devType = Device_GetName(nwkAddr);
    if (strcmp(devType, "Unknown Device") != 0) {
      LOG_INFO("✨ [REJOIN] Known %s joined: short=0x%04X, "
               "IEEE=%02x%02x%02x%02x%02x%02x%02x%02x\n",
               devType, nwkAddr, ieeeBytes[7], ieeeBytes[6], ieeeBytes[5], ieeeBytes[4],
               ieeeBytes[3], ieeeBytes[2], ieeeBytes[1], ieeeBytes[0]);
    } else {
      LOG_INFO("✨ [JOIN] New device joined: short=0x%04X, "
               "IEEE=%02x%02x%02x%02x%02x%02x%02x%02x\n",
               nwkAddr, ieeeBytes[7], ieeeBytes[6], ieeeBytes[5], ieeeBytes[4],
               ieeeBytes[3], ieeeBytes[2], ieeeBytes[1], ieeeBytes[0]);
    }

    Device_AddDiscoveredIeee(nwkAddr, ieeeBytes);
    Main_SetOnline(nwkAddr);

    // Throttle the discovery burst: a device that re-announces repeatedly
    // (e.g. a flaky/flooding siren) must NOT re-run this blocking sequence
    // every time, or it stalls the dispatcher loop and the logs "hang".
    if (Device_ShouldQuery(nwkAddr)) {

      // Trigger discovery and IEEE address resolution. The real responses
      // arrive asynchronously as events; we only need the SRSP ack here.
      uint8_t reqPay[4] = {nwkAddr & 0xFF, (nwkAddr >> 8) & 0xFF, 0x01, 0x00};
      ZNP_Sreq(0x25, 0x01, reqPay, 4, NULL, 3000); // IEEE_addr_req
      usleep(250000);
      
      ZNP_ZdoActiveEpReq(nwkAddr);
      usleep(250000);
      
      ZNP_QuerySimpleDesc(nwkAddr, 1); // Standard ep-1 (Okos, Aqara, Tuya, etc.)
      usleep(250000);
      
      ZNP_QuerySimpleDesc(nwkAddr, 43); // Develco/Frient siren ep-43 fallback
    }
  }
  // 1.5 ZDO State Change Indication (cmd0: 0x45, cmd1: 0xC0)
  else if (frame_->cmd0 == 0x45 && frame_->cmd1 == 0xC0) {
    if (frame_->len >= 1) {
      uint8_t currentState = frame_->payload[0];
      LOG_DEBUG("[STATE] Coordinator state change: %d\n", currentState);
    }
  }
  // 1.6 ZDO Leave Indication (cmd0: 0x45, cmd1: 0xCB)
  else if (frame_->cmd0 == 0x45 && frame_->cmd1 == 0xCB) {
    if (frame_->len >= 13) {
      uint16_t shortAddr = frame_->payload[0] | (frame_->payload[1] << 8);
      uint8_t request = frame_->payload[10];
      uint8_t rejoin = frame_->payload[12];

      // If request == 1 and rejoin == 0, it often means the Trust Center kicked
      // the device (e.g., due to failing Install Code authentication).
      if (request == 1 && rejoin == 0) {
        LOG_WARNING("❌ [REJECTED] Device 0x%04X was removed/rejected by Trust "
                    "Center (Install Code mismatch or key exchange failed)!\n",
                    shortAddr);
      } else {
        LOG_DEBUG("[LEAVE] Device 0x%04X intentionally left the network "
                  "(request=%d, rejoin=%d).\n",
                  shortAddr, request, rejoin);
      }
    } else if (frame_->len >= 10) {
      uint16_t shortAddr = frame_->payload[0] | (frame_->payload[1] << 8);
      LOG_DEBUG("[LEAVE] Device 0x%04X left the network!\n", shortAddr);
    }
  }
  // 1.7 ZDO TC Device Indication (cmd0: 0x45, cmd1: 0xCA)
  else if (frame_->cmd0 == 0x45 && frame_->cmd1 == 0xCA) {
    if (frame_->len >= 12) {
      uint16_t shortAddr = frame_->payload[0] | (frame_->payload[1] << 8);
      LOG_DEBUG("🔒 [TC_AUTH] Trust Center processing authentication/key "
                "exchange for Device 0x%04X...\n",
                shortAddr);
    }
  }
  // 1.8 APP_CNF BDB Indications (cmd0: 0x2F)
  else if (frame_->cmd0 == 0x2F) {
    // 0x80 = MT_APP_CNF_BDB_COMMISSIONING_NOTIFICATION
    if (frame_->cmd1 == 0x80 && frame_->len >= 2) {
      uint8_t status = frame_->payload[0];
      uint8_t commMode = frame_->payload[1];
      LOG_DEBUG("🛡️ [BDB_COMMISSIONING_NOTIFY] Status: %d, Mode: %d\n", status,
                commMode);
    }
    // 0x81 = MT_APP_CNF_BDB_TC_LINK_KEY_EXCHANGE_NOTIFICATION_IND
    else if (frame_->cmd1 == 0x81 && frame_->len >= 2) {
      uint8_t status = frame_->payload[0];
      LOG_WARNING("❌ [KEY_EXCHANGE_FAIL] Trust Center rejected a device! Key "
                  "Exchange Failed (Status: %d)\n",
                  status);
    }
  }
  // 2. ZDO Response/Callback Parser (0x45 0xFF, 0x45 0x81, 0x45 0x86)
  else if (frame_->cmd0 == 0x45 &&
           (frame_->cmd1 == 0xFF || frame_->cmd1 == 0x81 ||
            frame_->cmd1 == 0x86)) {
    uint8_t status = 0;
    uint16_t shortAddr = 0;
    uint16_t clusterId = 0;
    uint16_t srcAddr = 0;

    int matchCount = 0;
    uint8_t matchList[32];

    const uint8_t *asdu = NULL;
    int asduLen = 0;

    if (frame_->cmd1 == 0xFF) {
      if (frame_->len >= 9) {
        srcAddr = frame_->payload[0] | (frame_->payload[1] << 8);
        clusterId = frame_->payload[3] | (frame_->payload[4] << 8);
        asdu = &frame_->payload[9];
        asduLen = frame_->len - 9;

        if (clusterId == 0x8001) // IEEE_addr_rsp
        {
          if (asduLen >= 11) {
            status = asdu[0];
            if (status == 0) {
              shortAddr = asdu[9] | (asdu[10] << 8);
              Device_AddDiscoveredIeee(shortAddr, &asdu[1]);
            }
          }
        } else if (clusterId == 0x8004) // Simple_desc_rsp
        {
          if (asduLen >= 4) {
            status = asdu[0];
            shortAddr = asdu[1] | (asdu[2] << 8);
          }
        } else if (clusterId == 0x8005) // Active_EP_rsp
        {
          if (asduLen >= 4) {
            status = asdu[0];
            shortAddr = asdu[1] | (asdu[2] << 8);
            if (status == 0) {
              matchCount = asdu[3];
              if (matchCount > 32) {
                matchCount = 32;
              }
              memcpy(matchList, &asdu[4], matchCount);
            }
          }
        } else if (clusterId == 0x8006) // Match_Desc_rsp
        {
          if (asduLen >= 4) {
            status = asdu[0];
            shortAddr = asdu[1] | (asdu[2] << 8);
            if (status == 0) {
              matchCount = asdu[3];
              if (matchCount > 32) {
                matchCount = 32;
              }
              memcpy(matchList, &asdu[4], matchCount);
            }
          }
        }
      }
    } else if (frame_->cmd1 == 0x81) // Direct IEEE_addr_rsp indication
    {
      clusterId = 0x8001;
      if (frame_->len >= 11) {
        status = frame_->payload[0];
        if (status == 0) {
          shortAddr = frame_->payload[9] | (frame_->payload[10] << 8);
          Device_AddDiscoveredIeee(shortAddr, &frame_->payload[1]);
        }
      }
    } else if (frame_->cmd1 == 0x86) // Direct Match_desc_rsp indication
    {
      clusterId = 0x8006;
      if (frame_->len >= 4) {
        status = frame_->payload[0];
        shortAddr = frame_->payload[1] | (frame_->payload[2] << 8);
        if (status == 0) {
          matchCount = frame_->payload[3];
          if (matchCount > 32) {
            matchCount = 32;
          }
          memcpy(matchList, &frame_->payload[4], matchCount);
        }
      }
    }

    // Join trigger via MSG_CB (cluster 0x0013)
    if (clusterId == 0x0013) {
      const char *devType = Device_GetName(srcAddr);
      if (strcmp(devType, "Unknown Device") != 0) {
        LOG_DEBUG("✨ [REJOIN via MSG_CB] Known %s announced: short=0x%04X\n",
                 devType, srcAddr);
      } else {
        LOG_DEBUG("✨ [JOIN via MSG_CB] New device announced: short=0x%04X\n",
                 srcAddr);
      }
      if (asdu != NULL && asduLen >= 10) {
        uint16_t annceShort = asdu[0] | (asdu[1] << 8);
        Device_AddDiscoveredIeee(annceShort, &asdu[2]);
        Main_SetOnline(annceShort);
      }

      // Throttle the discovery burst so a repeatedly re-announcing device
      // cannot stall the dispatcher loop (see the 0xC1 handler above).
      // Exception: known Onics buttons re-announce after panic mode activation
      // reset — always re-discover them so we can see the newly unlocked EP
      // 0x23.
#if ENABLE_ONICS_BUTTON
      bool isKnownOnics = OnicsButton_IsKnown(srcAddr);
#else
      bool isKnownOnics = false;
#endif
      if (Device_ShouldQuery(srcAddr) || isKnownOnics) {
        uint8_t reqPay[4] = {srcAddr & 0xFF, (srcAddr >> 8) & 0xFF, 0x01, 0x00};
        ZNP_Sreq(0x25, 0x01, reqPay, 4, NULL, 3000);
        ZNP_ZdoActiveEpReq(srcAddr);
        ZNP_QuerySimpleDesc(srcAddr, 1);
        ZNP_QuerySimpleDesc(srcAddr, 43);

#if ENABLE_ONICS_BUTTON
        if (isKnownOnics) {
          // Hand the (slow) rebind sequence to the Onics worker thread;
          // blocking here would stall the dispatcher and drop AF frames.
          LOG_DEBUG("[Onics] Known button re-announced post-reset. Queueing "
                    "rebind...\n");
          OnicsButton_PostRebind(srcAddr);
        }
#endif
      }
    } else if (clusterId == 0x8001 && status == 0) {
      uint8_t ieee[8];
      if (Device_GetDiscoveredIeee(shortAddr, ieee)) {
        LOG_DEBUG("ZDO IEEE Rsp: short=0x%04X -> IEEE=", shortAddr);
        for (int i = 7; i >= 0; i--) {
          LOG_DEBUG_RAW("%02x", ieee[i]);
        }
        LOG_DEBUG_RAW("\n");

        // Update modules with the resolved IEEE address
#if ENABLE_SIREN
        Siren_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_AQARA_BUTTON
        AqaraButton_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_ONICS_BUTTON
        OnicsButton_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_AQARA_OCCUPANCY
        AqaraOccupancy_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_CONTACT_SENSOR
        ContactSensor_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_OKOS_SIREN
        OkosSiren_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_VIBRATION_SENSOR
        VibrationSensor_UpdateIeee(shortAddr, ieee);
#endif
#if ENABLE_AQARA_TVOC
        AqaraTvoc_UpdateIeee(shortAddr, ieee);
#endif
      }
    } else if (clusterId == 0x8005 && status == 0) {
      LOG_DEBUG("ZDO Active EPs Rsp: short=0x%04X, EPs=[", shortAddr);
      bool hasEp23 = false;
      for (int i = 0; i < matchCount; i++) {
        LOG_DEBUG_RAW("%d%s", matchList[i], (i == matchCount - 1) ? "" : ", ");
        if (matchList[i] == 0x23) {
          hasEp23 = true;
        }
      }
      LOG_DEBUG_RAW("]\n");
      for (int i = 0; i < matchCount; i++) {
        ZNP_QuerySimpleDesc(shortAddr, matchList[i]);
      }

#if ENABLE_ONICS_BUTTON
      // Backup: if we missed the announce but got the Active EP response,
      // we can re-bind here. But we already attempt it in the announce handler.
      if (hasEp23 && OnicsButton_IsKnown(shortAddr)) {
        // Repeating the bind is safe and ensures reliability, but it must
        // run on the Onics worker thread, not the dispatcher.
        LOG_DEBUG(
            "[Onics] EP 0x23 detected in Active EPs. Queueing rebind...\n");
        OnicsButton_PostRebind(shortAddr);
      }
#endif
    } else if (clusterId == 0x8004 && asdu != NULL && asduLen >= 11) {
      status = asdu[0];
      shortAddr = asdu[1] | (asdu[2] << 8);
      uint8_t epLen = asdu[3];
      if (status == 0 && asduLen >= 4 + epLen) {
        uint8_t ep = asdu[4];
        uint16_t profileId = asdu[5] | (asdu[6] << 8);
        uint16_t deviceId = asdu[7] | (asdu[8] << 8);
        uint8_t numIn = asdu[10];

        int offset = 11;
        uint16_t inCls[32];
        int numInCls = 0;
        for (int i = 0; i < numIn; i++) {
          if (offset + 2 <= asduLen) {
            inCls[numInCls++] = asdu[offset] | (asdu[offset + 1] << 8);
          }
          offset += 2;
        }

        int numOutCls = 0;
        uint16_t outCls[32];
        if (offset < asduLen) {
          uint8_t numOut = asdu[offset];
          offset += 1;
          for (int i = 0; i < numOut; i++) {
            if (offset + 2 <= asduLen) {
              outCls[numOutCls++] = asdu[offset] | (asdu[offset + 1] << 8);
            }
            offset += 2;
          }
        }

        LOG_DEBUG(
            "Device 0x%04X ep 0x%02X Profile=0x%04X DevID=0x%04X InClusters=[",
            shortAddr, ep, profileId, deviceId);
        for (int i = 0; i < numInCls; i++) {
          LOG_DEBUG("0x%04X%s", inCls[i], (i == numInCls - 1) ? "" : ", ");
        }
        LOG_DEBUG("] OutClusters=[");
        for (int i = 0; i < numOutCls; i++) {
          LOG_DEBUG("0x%04X%s", outCls[i], (i == numOutCls - 1) ? "" : ", ");
        }
        LOG_DEBUG("]\n");

        if (profileId == 0x0104) {
          bool isSiren = false;
          bool isOkosTuya = false;
          bool isContact = false;
          bool isVibration = false;
          bool isOnics = false;
          bool isAqara = false;
          bool isOccupancy = false;
          bool isTvoc = false;

          for (int i = 0; i < numInCls; i++) {
            if (inCls[i] == 0x0502) isSiren = true;
            if (inCls[i] == 0xEF00 || deviceId == 0x0051) isOkosTuya = true;
            if (inCls[i] == 0x000F || inCls[i] == 0x0012) isOnics = true;
            if (inCls[i] == 0x0006 || inCls[i] == 0x0012) isAqara = true;
            if (inCls[i] == 0x0406) isOccupancy = true;
            if (inCls[i] == 0x000C) isTvoc = true;
            if (inCls[i] == 0xFC04 || inCls[i] == 0x0101) isVibration = true;
          }
          
          for (int i = 0; i < numOutCls; i++) {
            if (outCls[i] == 0x0006) isAqara = true;
          }

          if (deviceId == 0x0107) isOccupancy = true;
          if (deviceId == 0x0228 || deviceId == 0x022D || deviceId == 0x0101) isVibration = true;

          if (deviceId == 0x0402 && !isVibration && !isTvoc) {
             LOG_DEBUG("Device 0x%04X is DevID 0x0402 (IAS Zone). Querying ModelIdentifier to classify...\n", shortAddr);
             uint8_t req[5] = {0x00, 0x55, 0x00, 0x05, 0x00};
             ZNP_AfDataRequestExt(2, shortAddr, ep, 0, 8, 0x0000, 0x55, 0, 30, req, 5);
          }

          if (isOccupancy) {
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_Discover(shortAddr, ep);
#endif
          } else if (isTvoc) {
#if ENABLE_AQARA_TVOC
            AqaraTvoc_Discover(shortAddr, ep);
#endif
          } else if (isContact) {
#if ENABLE_CONTACT_SENSOR
            ContactSensor_Discover(shortAddr, ep);
#endif
          } else if (isVibration) {
#if ENABLE_VIBRATION_SENSOR
            VibrationSensor_Discover(shortAddr, ep);
#endif
          } else if (isOkosTuya) {
#if ENABLE_OKOS_SIREN
            OkosSiren_Discover(shortAddr, ep);
#endif
          } else if (isSiren) {
#if ENABLE_OKOS_SIREN
            if (ep == 0x01 || !ENABLE_SIREN) OkosSiren_Discover(shortAddr, ep);
#endif
#if ENABLE_SIREN
            if (ep == 0x2B || !ENABLE_OKOS_SIREN) Siren_Discover(shortAddr, ep);
#endif
          } else if (isOnics) {
#if ENABLE_ONICS_BUTTON
            OnicsButton_Discover(shortAddr, ep);
#endif
          } else if (isAqara) {
#if ENABLE_AQARA_BUTTON
            AqaraButton_Discover(shortAddr, ep);
#endif
          }
        }
      }
    } else if (clusterId == 0x8006 && status == 0) {
      LOG_DEBUG("ZDO Match Desc Rsp: short=0x%04X, endpoints=[", shortAddr);
      for (int i = 0; i < matchCount; i++) {
        LOG_DEBUG_RAW("%d%s", matchList[i], (i == matchCount - 1) ? "" : ", ");
      }
      LOG_DEBUG_RAW("]\n");
      for (int i = 0; i < matchCount; i++) {
        ZNP_QuerySimpleDesc(shortAddr, matchList[i]);
      }
    }
  }
  // 3. AF Incoming Message
  else if (frame_->cmd0 == 0x44 && frame_->cmd1 == 0x81) {
    if (frame_->len < 17) {
      return;
    }

    AF_MSG_T af;
    af.clusterId = frame_->payload[2] | (frame_->payload[3] << 8);
    af.srcAddr = frame_->payload[4] | (frame_->payload[5] << 8);
    af.srcEp = frame_->payload[6];
    af.transSeq = frame_->payload[15];
    af.dataLen = frame_->payload[16];
    memcpy(af.data, &frame_->payload[17], af.dataLen);

    if (af.srcAddr == 0x0000)
      return;

    if (af.clusterId != 0xFCC0 && af.clusterId != 0x0400 &&
        af.clusterId != 0x0406 && af.clusterId != 0x0500) {
      LOG_DEBUG(
          "[MSG] Incoming AF Msg: src=0x%04X ep=0x%02X cluster=0x%04X len=%d\n",
          af.srcAddr, af.srcEp, af.clusterId, af.dataLen);
    }

    bool isKnown = false;
#if ENABLE_SIREN
    if (Siren_IsKnown(af.srcAddr)) {
      isKnown = true;
      Siren_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_AQARA_BUTTON
    if (AqaraButton_IsKnown(af.srcAddr)) {
      isKnown = true;
      AqaraButton_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_ONICS_BUTTON
    if (OnicsButton_IsKnown(af.srcAddr)) {
      isKnown = true;
      OnicsButton_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_AQARA_OCCUPANCY
    if (AqaraOccupancy_IsKnown(af.srcAddr)) {
      isKnown = true;
      AqaraOccupancy_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_CONTACT_SENSOR
    if (ContactSensor_IsKnown(af.srcAddr)) {
      isKnown = true;
      ContactSensor_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_VIBRATION_SENSOR
    if (VibrationSensor_IsKnown(af.srcAddr)) {
      isKnown = true;
      VibrationSensor_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_OKOS_SIREN
    if (OkosSiren_IsKnown(af.srcAddr)) {
      isKnown = true;
      OkosSiren_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif
#if ENABLE_AQARA_TVOC
    if (AqaraTvoc_IsKnown(af.srcAddr)) {
      isKnown = true;
      AqaraTvoc_UpdateSeen(af.srcAddr);
      Main_SetOnline(af.srcAddr);
    }
#endif

    if (!isKnown && af.clusterId == 0x0000 && af.dataLen >= 6) {
      uint8_t fc = af.data[0];
      int hdrLen = (fc & 0x04) ? 5 : 3;
      if (af.dataLen > hdrLen) {
        uint8_t cmdId = af.data[hdrLen - 1];
        if (cmdId == 0x01 || cmdId == 0x0A) // Read Rsp or Report
        {
          const uint8_t *zcl = &af.data[hdrLen];
          uint16_t attr = zcl[0] | (zcl[1] << 8);

          if (attr == 0x0005) {
            uint8_t typeOffset = (cmdId == 0x01) ? 3 : 2;
            // For Read Rsp (0x01), zcl[2] is Status. If success (0x00), type is
            // at 3. For Report (0x0A), there is no Status, type is at 2.
            bool success = (cmdId == 0x0A) || (cmdId == 0x01 && zcl[2] == 0x00);

            if (success && zcl[typeOffset] == 0x42) {
              uint8_t strLen = zcl[typeOffset + 1];
              if (af.dataLen >= hdrLen + typeOffset + 2 + strLen) {
                char model[64] = {0};
                memcpy(model, &zcl[typeOffset + 2],
                       (strLen < 63) ? strLen : 63);
                LOG_DEBUG("Device 0x%04X reported ModelIdentifier: '%s'\n",
                          af.srcAddr, model);
                if (strstr(model, "airmonitor")) {
#if ENABLE_AQARA_TVOC
                  AqaraTvoc_Discover(af.srcAddr, af.srcEp);
                  isKnown = true;
#endif
                } else if (strstr(model, "magnet") ||
                           strstr(model, "sensor_switch")) {
#if ENABLE_CONTACT_SENSOR
                  ContactSensor_Discover(af.srcAddr, af.srcEp);
                  isKnown = true;
#endif
                }
              }
            }
          }
        }
      }
    }

    if (!isKnown && Device_ShouldQuery(af.srcAddr)) {
      if (af.srcEp == 43) {
#if ENABLE_SIREN
        Siren_Discover(af.srcAddr, af.srcEp);
#endif
      } else if (af.srcEp == 1) {
#if ENABLE_OKOS_SIREN
        OkosSiren_Discover(af.srcAddr, af.srcEp);
#endif
      } else {
        LOG_DEBUG("❓ Unknown device 0x%04X sent AF message on cluster 0x%04X. "
                  "Requesting Active EPs...\n",
                  af.srcAddr, af.clusterId);
        ZNP_ZdoActiveEpReq(af.srcAddr);
      }
    }

#if ENABLE_OKOS_SIREN
    if (OkosSiren_IsKnown(af.srcAddr)) {
      OkosSiren_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_SIREN
    else if (Siren_IsKnown(af.srcAddr)) {
      Siren_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_AQARA_BUTTON
    else if (AqaraButton_IsKnown(af.srcAddr)) {
      AqaraButton_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_ONICS_BUTTON
    else if (OnicsButton_IsKnown(af.srcAddr)) {
      OnicsButton_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_CONTACT_SENSOR
    else if (ContactSensor_IsKnown(af.srcAddr)) {
      ContactSensor_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_VIBRATION_SENSOR
    else if (VibrationSensor_IsKnown(af.srcAddr)) {
      VibrationSensor_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_AQARA_OCCUPANCY
    else if (AqaraOccupancy_IsKnown(af.srcAddr)) {
      AqaraOccupancy_PostAf(af.srcAddr, &af);
    }
#endif
#if ENABLE_AQARA_TVOC
    else if (AqaraTvoc_IsKnown(af.srcAddr)) {
      AqaraTvoc_PostAf(af.srcAddr, &af);
    }
#endif
  }
}
