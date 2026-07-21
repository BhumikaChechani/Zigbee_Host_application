///
/// @file   siren.h
/// @brief  Smart Siren (SIRZB-110, IAS WD 0x0502) module: registry + worker thread.
///
/// Owns the list of known sirens and a worker thread. The dispatcher registers
/// sirens and routes their (rare) traffic here; the use-case layer calls
/// Siren_ControlAll() to actually sound/silence them. All siren device I/O runs
/// on the siren thread so it never blocks the dispatcher or other sensors.
///
#ifndef SIREN_H
#define SIREN_H

#include "znp_host.h"
#include "sensor_common.h"

/// @brief One registered siren.
typedef struct
{
    uint16_t shortAddr; ///< 16-bit network address.
    uint8_t endpoint;   ///< Endpoint hosting the IAS WD cluster (typically 0x2B).
    double lastSeen;    ///< Timestamp (seconds) of the last frame from this siren.
    uint8_t ieee[8];    ///< 64-bit IEEE address (little-endian).
    bool hasIeee;       ///< True once @ref ieee is known.
    uint8_t zoneId;     ///< IAS zone id assigned to the siren's tamper zone.
    bool configured;    ///< True once CIE address has been written (setup done).
    uint8_t volume;     ///< Per-device volume level (0-3).
    uint8_t mode;       ///< Per-device warning mode (1-6).
} SIREN_T;

#define MAX_SIRENS 32   ///< Maximum sirens tracked.

extern SIREN_T g_sirens[MAX_SIRENS]; ///< Siren registry.
extern int g_numSirens;              ///< Number of valid entries in @ref g_sirens.

/// @brief  Initialize the siren registry and inbox. Call once at startup.
/// @return None.
void Siren_Init( void );

/// @brief  Spawn the siren worker thread.
/// @return None.
void Siren_Start( void );

/// @brief  Queue an ASSIGN work item so the siren thread runs setup for a device.
/// @param  shortAddr_  Siren network address.
/// @return None.
void Siren_PostAssign( uint16_t shortAddr_ );

/// @brief  Queue an AF message from a siren for the siren thread to handle.
/// @param  shortAddr_  Siren network address.
/// @param  af_         The decoded AF message.
/// @return None.
void Siren_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ );

///
/// @brief  Register (or refresh) a siren in the registry (dispatcher context).
///
/// Only touches shared state and then hands setup off to the siren thread via
/// Siren_PostAssign(); it performs no blocking device I/O itself.
///
/// @param  shortAddr_  Siren network address.
/// @param  endpoint_   Endpoint hosting the IAS WD cluster.
/// @return None.
///
void Siren_Discover( uint16_t shortAddr_, uint8_t endpoint_ );

///
/// @brief  Record a siren's resolved IEEE and collapse stale duplicate entries.
/// @param  shortAddr_  Siren network address.
/// @param  ieee_       Resolved 8-byte IEEE address.
/// @return None.
///
void Siren_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ );

///
/// @brief  Per-device setup (siren thread): resolve IEEE then write CIE address.
/// @param  shortAddr_  Siren network address. Idempotent via the configured flag.
/// @return None.
///
void Siren_Setup( uint16_t shortAddr_ );

///
/// @brief  Handle a siren tamper-zone enroll request by assigning an id + reply.
/// @param  shortAddr_  Siren network address.
/// @param  endpoint_   Source endpoint.
/// @param  transSeq_   ZCL transaction sequence number to echo.
/// @param  zoneType_   Reported IAS zone type.
/// @return None.
///
void Siren_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ );

///
/// @brief  Start or stop every registered siren.
/// @param  warnMode_  0 = stop, non-zero = start (burglar warning).
/// @return None.
///
void Siren_ControlAll( uint8_t warnMode_ );

/// @brief Start or stop a specific siren.
/// @param shortAddr_ Siren network address.
/// @param warnMode_ 0 = stop, non-zero = start.
void Siren_Control( uint16_t shortAddr_, uint8_t warnMode_ );

/// @brief Start or stop sirens for a specific duration in seconds using per-device config.
void Siren_ControlAllDuration( uint8_t warnMode_, uint16_t durationSeconds_ );

/// @brief Explicitly trigger all sirens with a specific mode and volume.
/// @param mode_ 0=stop, 1-6=start with mode.
/// @param volume_ 0-3 volume level.
/// @param durationSeconds_ duration.
void Siren_TriggerAll( uint8_t mode_, uint8_t volume_, uint16_t durationSeconds_ );

/// @brief Send a short Squawk/Chime to every registered siren.
/// @param squawkMode_ e.g., 0=Armed, 1=Disarmed.
/// @param squawkLevel_ e.g., 0=Low, 1=Medium, 2=High, 3=Very High.
void Siren_ControlSquawk( uint8_t squawkMode_, uint8_t squawkLevel_ );

/// @brief Set a specific siren's volume.
/// @param shortAddr_ Siren network address.
/// @param volume_ 0=low, 1=medium, 2=high, 3=very high.
void Siren_SetVolume( uint16_t shortAddr_, uint8_t volume_ );

/// @brief Emulate a specific number of short beeps by rapidly toggling Start Warning.
/// @param count_ Number of beeps.
void Siren_Beep( int count_ );

/// @brief  Queue a beep sequence on the siren worker thread (non-blocking).
///         Use this from the use-case thread instead of Siren_Beep() so the
///         beep's sleeps never delay other event handling.
/// @param  count_ Number of beeps.
void Siren_PostBeep( int count_ );

/// @brief Get the volume for a specific siren.
/// @param shortAddr_ Siren network address.
/// @return Current volume level.
uint8_t Siren_GetVolume( uint16_t shortAddr_ );

/// @brief Set a specific siren's mode.
/// @param shortAddr_ Siren network address.
/// @param mode_ 1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic
void Siren_SetMode( uint16_t shortAddr_, uint8_t mode_ );

/// @brief Get the mode for a specific siren.
/// @param shortAddr_ Siren network address.
/// @return Current mode level.
uint8_t Siren_GetMode( uint16_t shortAddr_ );

/// @brief Read environment data (battery, temp) from a specific siren.
/// @param shortAddr_ Target network address.
void Siren_ReadEnvironment( uint16_t shortAddr_ );

/// @brief  Print the registered sirens (for the CLI 'status' command).
/// @return None.
void Siren_PrintStatus( void );

/// @brief  Is @p shortAddr_ a known siren? Thread-safe.
/// @param  shortAddr_  Device network address.
/// @return true if registered as a siren.
bool Siren_IsKnown( uint16_t shortAddr_ );

/// @brief Get the endpoint for a known siren.
/// @param shortAddr_ Device network address.
/// @return Endpoint (e.g. 0x2B), or 0 if not found.
uint8_t Siren_GetEndpoint( uint16_t shortAddr_ );

/// @brief  Update a siren's last-seen timestamp.
/// @param  shortAddr_  Siren network address.
/// @return None.
void Siren_UpdateSeen( uint16_t shortAddr_ );

/// @brief  Re-enumerate endpoints of all known sirens (CLI 'discover').
/// @return None.
void Siren_DiscoverAllActiveEp( void );

#endif // SIREN_H
