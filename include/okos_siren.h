///
/// @file   okos_siren.h
/// @brief  Okos Smart Siren (Tuya/Zigbee 3.0 IAS WD) module: registry + worker thread.
///
/// The Okos Smart Siren is a Tuya-ecosystem IAS Warning Device that communicates
/// over Zigbee 3.0.  It differs from the Frient SIRZB-110 in several ways:
///
///  - It exposes a built-in temperature + humidity sensor (clusters 0x0402, 0x0405).
///  - It supports 18 selectable alarm tones via Manufacturer-Specific attributes
///    (Tuya cluster 0xEF00 or a Tuya-specific attribute inside cluster 0x0502).
///  - It has an independently controllable RGB/white strobe (Start Warning byte 1).
///  - It supports battery backup (2 × CR123A) reported on Power Config cluster 0x0001.
///  - Its default endpoint is typically 0x01 (not 0x2B as on the Frient).
///  - Volume levels map to: 0=Low (~70 dB), 1=Medium (~85 dB), 2=High (~100 dB).
///
/// This module owns a dedicated worker thread that handles:
///   1. IAS Zone enroll handshake (cluster 0x0500, cmd 0x01).
///   2. Zone Status Change notifications — tamper-bit detection.
///   3. Temperature / humidity attribute reports.
///   4. Battery-level reports from the Power Configuration cluster.
///
/// Integration status (see OKOS_SIREN_FEATURES.md for the full feature list):
///   [x] Discovery & registration
///   [x] IAS CIE address write (pairing)
///   [x] Zone enroll response
///   [ ] Alarm tone selection (18 tones via Tuya cluster)  ← TODO phase 2
///   [ ] Strobe-only mode                                  ← TODO phase 2
///   [ ] Temperature / humidity read                       ← TODO phase 3
///   [ ] OTA firmware upgrade                              ← TODO phase 4
///
#ifndef OKOS_SIREN_H
#define OKOS_SIREN_H

#include "znp_host.h"
#include "sensor_common.h"

// ---------------------------------------------------------------------------
// Tuya / Okos-specific ZCL constants
// ---------------------------------------------------------------------------

/// Tuya manufacturer-specific cluster used for advanced tone / mode control.
#define OKOS_TUYA_CLUSTER       0xEF00U

/// IAS Warning Device cluster (standard).
#define OKOS_IAS_WD_CLUSTER     0x0502U

/// IAS Zone cluster (standard).
#define OKOS_IAS_ZONE_CLUSTER   0x0500U

/// Power Configuration cluster (battery).
#define OKOS_POWER_CLUSTER      0x0001U

/// Temperature Measurement cluster.
#define OKOS_TEMP_CLUSTER       0x0402U

/// Relative Humidity Measurement cluster.
#define OKOS_HUM_CLUSTER        0x0405U

// ---------------------------------------------------------------------------
// Alarm tone IDs (Okos / Tuya specific, 1-based, 1..18)
// ---------------------------------------------------------------------------
typedef enum
{
    OKOS_TONE_BURGLAR          =  1,  ///< Classic burglar alarm.
    OKOS_TONE_FIRE             =  2,  ///< Fire alarm.
    OKOS_TONE_EMERGENCY        =  3,  ///< Generic emergency.
    OKOS_TONE_POLICE_PANIC     =  4,  ///< Police panic.
    OKOS_TONE_FIRE_PANIC       =  5,  ///< Fire panic.
    OKOS_TONE_EMERGENCY_PANIC  =  6,  ///< Emergency panic.
    OKOS_TONE_DOORBELL_1       =  7,  ///< Doorbell chime 1.
    OKOS_TONE_DOORBELL_2       =  8,  ///< Doorbell chime 2.
    OKOS_TONE_BEEP_FAST        =  9,  ///< Fast beep (arm/disarm acknowledgement).
    OKOS_TONE_BEEP_SLOW        = 10,  ///< Slow beep.
    OKOS_TONE_SIREN_HIGH       = 11,  ///< High-pitch continuous siren.
    OKOS_TONE_SIREN_LOW        = 12,  ///< Low-pitch continuous siren.
    OKOS_TONE_SWEEP            = 13,  ///< Frequency sweep siren.
    OKOS_TONE_PULSE            = 14,  ///< Pulsed alert.
    OKOS_TONE_WARBLE           = 15,  ///< Warble tone.
    OKOS_TONE_CHIRP            = 16,  ///< Short chirp (squawk emulation).
    OKOS_TONE_CUCKOO           = 17,  ///< Cuckoo clock tone (novelty).
    OKOS_TONE_CUSTOM           = 18,  ///< Reserved / custom tone slot.
} OKOS_TONE_T;

// ---------------------------------------------------------------------------
// Strobe mode flags (byte 1 of Start Warning command, bits [7:6])
// ---------------------------------------------------------------------------
#define OKOS_STROBE_NONE        0x00U  ///< No strobe.
#define OKOS_STROBE_USE_LEVEL   0x04U  ///< Use Strobe Level field.

// ---------------------------------------------------------------------------
// Device record
// ---------------------------------------------------------------------------

/// @brief One registered Okos Smart Siren device.
typedef struct
{
    uint16_t shortAddr;       ///< 16-bit Zigbee network address.
    uint8_t  endpoint;        ///< Endpoint hosting IAS WD cluster (usually 0x01).
    double   lastSeen;        ///< Timestamp (seconds) of the last frame from this device.
    uint8_t  ieee[8];         ///< 64-bit IEEE address (little-endian).
    bool     hasIeee;         ///< True once @ref ieee has been resolved.
    uint8_t  zoneId;          ///< IAS zone id assigned during enroll.
    bool     configured;      ///< True once CIE address write has been sent.
    bool     isTampered;      ///< Current tamper state (bit 2 of zone status).
    bool     isAlarming;      ///< True while a Start Warning is active.

    // --- User-configurable settings (persisted in devices.txt) ---
    uint8_t  volume;          ///< 0=Low, 1=Medium, 2=High (maps to ZCL Siren Level).
    uint8_t  toneId;          ///< Alarm tone 1-18 (see OKOS_TONE_T).
    uint8_t  strobeMode;      ///< OKOS_STROBE_NONE or OKOS_STROBE_USE_LEVEL.

    // --- Live environmental readings (populated once feature is enabled) ---
    int16_t  temperatureCdeg; ///< Temperature in 0.01 °C units (0x7FFF = unknown).
    uint16_t humidityHpct;    ///< Humidity in 0.01 % units (0xFFFF = unknown).
    uint8_t  batteryPct;      ///< Battery percentage 0-100 (0xFF = unknown).
} OKOS_SIREN_T;

#define MAX_OKOS_SIRENS 16    ///< Maximum Okos sirens tracked simultaneously.

extern OKOS_SIREN_T g_okosSirens[MAX_OKOS_SIRENS]; ///< Okos Siren registry.
extern int          g_numOkosSirens;                ///< Valid entries in @ref g_okosSirens.

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

/// @brief Initialise the Okos siren registry and inbox.  Call once at startup.
void OkosSiren_Init( void );

/// @brief Spawn the Okos siren worker thread.
void OkosSiren_Start( void );

// ---------------------------------------------------------------------------
// Discovery & registry
// ---------------------------------------------------------------------------

/// @brief Register (or refresh) a siren in the registry.
/// @param shortAddr_  Device network address.
/// @param endpoint_   Endpoint hosting the IAS WD cluster.
void OkosSiren_Discover( uint16_t shortAddr_, uint8_t endpoint_ );

/// @brief Record a resolved IEEE address and collapse stale duplicates.
/// @param shortAddr_  Device network address.
/// @param ieee_       8-byte IEEE address.
void OkosSiren_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ );

// ---------------------------------------------------------------------------
// Worker-thread helpers (public only for the thread itself)
// ---------------------------------------------------------------------------

/// @brief Queue an ASSIGN work item so the siren thread runs setup.
/// @param shortAddr_  Device network address.
void OkosSiren_PostAssign( uint16_t shortAddr_ );

/// @brief Queue an AF message for the siren thread.
/// @param shortAddr_  Device network address.
/// @param af_         Decoded AF message.
void OkosSiren_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ );

/// @brief Run CIE-address setup on the siren thread (idempotent via configured flag).
/// @param shortAddr_  Device network address.
void OkosSiren_Setup( uint16_t shortAddr_ );

/// @brief Handle an IAS Zone Enroll Request from a device.
/// @param shortAddr_  Device network address.
/// @param endpoint_   Source endpoint.
/// @param transSeq_   ZCL transaction sequence number to echo.
/// @param zoneType_   Reported IAS zone type.
void OkosSiren_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_,
                             uint8_t transSeq_, uint16_t zoneType_ );

// ---------------------------------------------------------------------------
// Alarm control
// ---------------------------------------------------------------------------

/// @brief Start or stop all Okos sirens with their configured tone and volume.
/// @param warnMode_  0 = stop, non-zero = start.
void OkosSiren_ControlAll( uint8_t warnMode_ );

/// @brief Start or stop all Okos sirens for an explicit duration.
/// @param warnMode_        0 = stop, non-zero = start.
/// @param durationSeconds_ Warning duration in seconds (0 = stop immediately).
void OkosSiren_ControlAllDuration( uint8_t warnMode_, uint16_t durationSeconds_ );

/// @brief Start or stop a specific Okos siren.
/// @param shortAddr_  Device network address.
/// @param warnMode_   0 = stop, non-zero = start.
void OkosSiren_Control( uint16_t shortAddr_, uint8_t warnMode_ );



// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// @brief Set the volume for a specific Okos siren (persisted).
/// @param shortAddr_  Device network address.
/// @param volume_     0=Low, 1=Medium, 2=High.
void OkosSiren_SetVolume( uint16_t shortAddr_, uint8_t volume_ );

/// @brief Get the current volume for a specific Okos siren.
/// @param shortAddr_  Device network address.
/// @return Volume level 0-2 (default 2).
uint8_t OkosSiren_GetVolume( uint16_t shortAddr_ );

/// @brief Set the alarm tone for a specific Okos siren (1-18, persisted).
/// @param shortAddr_  Device network address.
/// @param toneId_     Tone identifier (OKOS_TONE_T, 1-18).
void OkosSiren_SetTone( uint16_t shortAddr_, uint8_t toneId_ );

/// @brief Get the configured alarm tone for a specific Okos siren.
/// @param shortAddr_  Device network address.
/// @return Tone id 1-18 (default OKOS_TONE_BURGLAR).
uint8_t OkosSiren_GetTone( uint16_t shortAddr_ );

/// @brief Set the strobe mode for a specific Okos siren (persisted).
/// @param shortAddr_  Device network address.
/// @param strobeMode_ OKOS_STROBE_NONE or OKOS_STROBE_USE_LEVEL.
void OkosSiren_SetStrobe( uint16_t shortAddr_, uint8_t strobeMode_ );

// ---------------------------------------------------------------------------
// Environmental sensor (Phase 3 — read on demand)
// ---------------------------------------------------------------------------

/// @brief Poll temperature and humidity from a specific Okos siren.
/// @param shortAddr_  Device network address.
void OkosSiren_ReadEnvironment( uint16_t shortAddr_ );

/// @brief Poll battery level from a specific Okos siren.
/// @param shortAddr_  Device network address.
void OkosSiren_ReadBattery( uint16_t shortAddr_ );

// ---------------------------------------------------------------------------
// Watchdog / keep-alive
// ---------------------------------------------------------------------------

/// @brief Poll all registered Okos sirens (called from the main loop, throttled).
void OkosSiren_PollAll( void );

// ---------------------------------------------------------------------------
// Status & utilities
// ---------------------------------------------------------------------------

/// @brief Print the Okos siren registry (CLI 'status' command).
void OkosSiren_PrintStatus( void );

/// @brief Query if a network address is a known Okos siren.
/// @param shortAddr_  Device network address.
/// @return true if registered.
bool OkosSiren_IsKnown( uint16_t shortAddr_ );

/// @brief Get the endpoint for a known Okos siren.
/// @param shortAddr_  Device network address.
/// @return Endpoint (e.g. 0x01), or 0 if not found.
uint8_t OkosSiren_GetEndpoint( uint16_t shortAddr_ );

/// @brief Update the last-seen timestamp for a known Okos siren.
/// @param shortAddr_  Device network address.
void OkosSiren_UpdateSeen( uint16_t shortAddr_ );

/// @brief Re-enumerate endpoints of all registered Okos sirens (CLI 'discover').
void OkosSiren_DiscoverAllActiveEp( void );

#endif // OKOS_SIREN_H
