///
/// @file   aqara_occupancy.h
/// @brief  Aqara Occupancy Sensor (Occupancy Sensing 0x0406) module: registry +
/// thread.
///
#ifndef AQARA_OCCUPANCY_H
#define AQARA_OCCUPANCY_H

#include "sensor_common.h"
#include "znp_host.h"

#define MAX_OCCUPANCY_ZONES 4

typedef struct {
  uint32_t minCm;
  uint32_t maxCm;
  bool isActive; ///< True if this zone index is configured.
  bool occupied; ///< Current presence state of this zone slice.
} OccupancyZone;

/// @brief Structure to track an Aqara occupancy sensor.
typedef struct {
  uint16_t shortAddr; ///< 16-bit network address.
  uint8_t endpoint;   ///< Endpoint hosting the occupancy sensing cluster.
  double lastSeen; ///< Timestamp (seconds) of the last frame from this device.
  uint8_t ieee[8]; ///< 64-bit IEEE address (little-endian).
  bool hasIeee;    ///< True once @ref ieee is known.
  bool configured; ///< True once setup/binding succeeded (verified).
  uint8_t setupRetries;     ///< Failed setup attempts (retry cap).
  bool absenceDelayApplied; ///< True once the absence-delay re-write was sent
                            ///< this boot.
  bool rawOccupied; ///< Raw state from the sensor (before distance filtering).
  uint32_t currentDistanceCm; ///< Last reported distance to target in cm.
  bool hasLightState; ///< True if we have received at least one light reading.
  bool isLightOn;     ///< Last known state of the light.
  uint16_t lastLightLevel; ///< Last reported raw light intensity level.
  uint16_t lightThreshold; ///< Per-device threshold for Day/Night detection.
  double lastPolled;  ///< Timestamp of the last outgoing poll (Read Attributes) sent to this device.
  double lastSetupAttempt; ///< Timestamp of the last setup attempt (rate-limiter).
  OccupancyZone zones[MAX_OCCUPANCY_ZONES]; ///< Configured distance ranges.
  uint8_t motionStatus;      ///< Last radar motion classification (attr 0x0143).
  uint8_t approachDirection; ///< Last approach direction (attr 0x0144). 0=Left, 1=Right, 0xFF=Unknown.
} AQARA_OCCUPANCY_T;

#define MAX_AQARA_OCCUPANCY 32 ///< Maximum occupancy sensors tracked.

extern AQARA_OCCUPANCY_T
    g_aqaraOccupancies[MAX_AQARA_OCCUPANCY]; ///< Occupancy registry.
extern int g_numAqaraOccupancies; ///< Valid entries in @ref g_aqaraOccupancies.

/// @brief  Initialize the Aqara occupancy registry and inbox. Call once at
/// startup.
/// @return None.
void AqaraOccupancy_Init(void);

/// @brief  Spawn the Aqara occupancy worker thread.
/// @return None.
void AqaraOccupancy_Start(void);

/// @brief  Queue an ASSIGN work item so the thread runs setup for a device.
/// @param  shortAddr_  Device network address.
/// @return None.
void AqaraOccupancy_PostAssign(uint16_t shortAddr_);

/// @brief  Queue an AF message for the Aqara occupancy thread to parse.
/// @param  shortAddr_  Device network address.
/// @param  af_         The decoded AF message.
/// @return None.
void AqaraOccupancy_PostAf(uint16_t shortAddr_, const AF_MSG_T *af_);

///
/// @brief  Register (or refresh) an occupancy sensor (dispatcher context).
/// @param  shortAddr_  Device network address.
/// @param  endpoint_   Endpoint hosting the occupancy sensing cluster.
/// @return None.
///
void AqaraOccupancy_Discover(uint16_t shortAddr_, uint8_t endpoint_);

///
/// @brief  Record a resolved IEEE and collapse stale duplicate entries.
/// @param  shortAddr_  Device network address.
/// @param  ieee_       Resolved 8-byte IEEE address.
/// @return None.
///
void AqaraOccupancy_UpdateIeee(uint16_t shortAddr_, const uint8_t *ieee_);

///
/// @brief  Per-device setup (occupancy thread): resolve IEEE then bind 0x0406.
/// @param  shortAddr_  Device network address.
/// @return None.
///
void AqaraOccupancy_Setup(uint16_t shortAddr_);

///
/// @brief  Map occupancy states to use-case events.
/// @param  shortAddr_  Device network address.
/// @param  occupied_   Occupancy state (1 = occupied, 0 = unoccupied).
/// @return None.
///
void AqaraOccupancy_HandleState(uint16_t shortAddr_, uint8_t occupied_);
void AqaraOccupancy_HandleDistance(uint16_t shortAddr_, uint32_t cm_);
void AqaraOccupancy_HandleLightState(uint16_t shortAddr_, uint16_t light_);
void AqaraOccupancy_SetZone(uint16_t shortAddr_, int zoneIdx_, uint32_t minCm_,
                            uint32_t maxCm_);
void AqaraOccupancy_DeleteZone(uint16_t shortAddr_, int zoneIdx_);
void AqaraOccupancy_SetHwDetectionRange(uint16_t shortAddr_, uint32_t bitmask_);
void AqaraOccupancy_SpatialLearning(uint16_t shortAddr_);
void AqaraOccupancy_SetSensitivity(uint16_t shortAddr_, uint8_t level_);
void AqaraOccupancy_ReadEnvironment(uint16_t shortAddr_);
void AqaraOccupancy_SetLightThreshold(uint16_t shortAddr_, uint16_t threshold_);

/// @brief  Print the registered occupancy sensors (CLI 'status').
/// @return None.
void AqaraOccupancy_PrintStatus(void);

/// @brief  Is @p shortAddr_ a known occupancy sensor? Thread-safe.
/// @param  shortAddr_  Device network address.
/// @return true if registered.
bool AqaraOccupancy_IsKnown(uint16_t shortAddr_);

///
/// @brief  Are ALL registered occupancy sensors currently reporting presence?
///
/// Implements the "every zone occupied" alarm condition. Thread-safe.
///
/// @return true if at least one sensor is registered and every one of them is
///         currently occupied; false otherwise.
///
bool AqaraOccupancy_AllOccupied(void);

///
/// @brief  Count how many registered occupancy zones are currently occupied.
/// @return Number of zones with presence (0..g_numAqaraOccupancies).
/// Thread-safe.
///
int AqaraOccupancy_OccupiedCount(void);

/// @brief  Update a device's last-seen timestamp.
/// @param  shortAddr_  Device network address.
/// @return None.
void AqaraOccupancy_UpdateSeen(uint16_t shortAddr_);

/// @brief  Re-enumerate endpoints of all known occupancy sensors (CLI
/// 'discover').
/// @return None.
void AqaraOccupancy_DiscoverAllActiveEp(void);

///
/// @brief  Actively read the presence attribute from every occupancy sensor.
///
/// Sends a ZCL Read Attributes for 0xFCC0/0x0142 to each registered sensor. A
/// Read Attributes Response proves the device is reachable/awake; silence means
/// it is asleep or has left the network. Used by the CLI 'poll' command.
///
/// @return None.
///
void AqaraOccupancy_PollAll(void);

#endif // AQARA_OCCUPANCY_H
