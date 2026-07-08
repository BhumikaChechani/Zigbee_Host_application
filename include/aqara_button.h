///
/// @file   aqara_button.h
/// @brief  Aqara Wireless Mini Switch T1 (On/Off 0x0006) module: registry + thread.
///
/// Owns the list of known Aqara switches and a worker thread. On assignment it
/// binds the switch's On/Off cluster to the coordinator; on a press it forwards
/// a high-level event to the use-case layer (it never drives the siren itself).
///
#ifndef AQARA_BUTTON_H
#define AQARA_BUTTON_H

#include "znp_host.h"
#include "sensor_common.h"

/// @brief One registered Aqara switch.
typedef struct
{
    uint16_t shortAddr; ///< 16-bit network address.
    uint8_t endpoint;   ///< Endpoint hosting the On/Off cluster.
    double lastSeen;    ///< Timestamp (seconds) of the last frame from this device.
    uint8_t ieee[8];    ///< 64-bit IEEE address (little-endian).
    bool hasIeee;       ///< True once @ref ieee is known.
    bool configured;    ///< True once the On/Off bind has been created.
} AQARA_BUTTON_T;

#define MAX_AQARA_BUTTONS 32 ///< Maximum Aqara switches tracked.

extern AQARA_BUTTON_T g_aqaraButtons[MAX_AQARA_BUTTONS]; ///< Aqara registry.
extern int g_numAqaraButtons;   ///< Valid entries in @ref g_aqaraButtons.

/// @brief  Initialize the Aqara registry and inbox. Call once at startup.
/// @return None.
void AqaraButton_Init( void );

/// @brief  Spawn the Aqara worker thread.
/// @return None.
void AqaraButton_Start( void );

/// @brief  Queue an ASSIGN work item so the thread runs setup for a device.
/// @param  shortAddr_  Device network address.
/// @return None.
void AqaraButton_PostAssign( uint16_t shortAddr_ );

/// @brief  Queue an AF message for the Aqara thread to parse.
/// @param  shortAddr_  Device network address.
/// @param  af_         The decoded AF message.
/// @return None.
void AqaraButton_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ );

///
/// @brief  Register (or refresh) an Aqara switch (dispatcher context).
/// @param  shortAddr_  Device network address.
/// @param  endpoint_   Endpoint hosting the On/Off cluster.
/// @return None.
///
void AqaraButton_Discover( uint16_t shortAddr_, uint8_t endpoint_ );

///
/// @brief  Record a resolved IEEE and collapse stale duplicate entries.
/// @param  shortAddr_  Device network address.
/// @param  ieee_       Resolved 8-byte IEEE address.
/// @return None.
///
void AqaraButton_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ );

///
/// @brief  Per-device setup (Aqara thread): resolve IEEE then bind On/Off.
/// @param  shortAddr_  Device network address. Idempotent via the configured flag.
/// @return None.
///
void AqaraButton_Setup( uint16_t shortAddr_ );

///
/// @brief  Map an On/Off command to a use-case event (on/off/toggle).
/// @param  shortAddr_  Device network address.
/// @param  cmdId_      ZCL On/Off command id (0 = off, 1 = on, 2 = toggle).
/// @return None.
///
void AqaraButton_HandleCommand( uint16_t shortAddr_, uint8_t cmdId_ );

/// @brief  Print the registered Aqara switches (CLI 'status').
/// @return None.
void AqaraButton_PrintStatus( void );

/// @brief  Is @p shortAddr_ a known Aqara switch? Thread-safe.
/// @param  shortAddr_  Device network address.
/// @return true if registered.
bool AqaraButton_IsKnown( uint16_t shortAddr_ );

/// @brief  Update a device's last-seen timestamp.
/// @param  shortAddr_  Device network address.
/// @return None.
void AqaraButton_UpdateSeen( uint16_t shortAddr_ );

/// @brief  Re-enumerate endpoints of all known Aqara switches (CLI 'discover').
/// @return None.
void AqaraButton_DiscoverAllActiveEp( void );

#endif // AQARA_BUTTON_H
