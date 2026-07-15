///
/// @file   onics_button.h
/// @brief  Onics SBTZB-110 Smart/Panic Button module: registry + worker thread.
///
/// Owns the list of known Onics buttons and a worker thread. On assignment it
/// binds On/Off, writes the CIE address, and sends the panic-activation write.
/// It answers IAS Zone enroll requests, and forwards presses and panic alarms
/// to the use-case layer (it never drives the siren directly).
///
#ifndef ONICS_BUTTON_H
#define ONICS_BUTTON_H

#include "znp_host.h"
#include "sensor_common.h"

/// @brief One registered Onics button.
typedef struct
{
    uint16_t shortAddr; ///< 16-bit network address.
    uint8_t endpoint;   ///< Endpoint hosting On/Off + IAS Zone clusters.
    double lastSeen;    ///< Timestamp (seconds) of the last frame from this device.
    uint8_t ieee[8];    ///< 64-bit IEEE address (little-endian).
    bool hasIeee;       ///< True once @ref ieee is known.
    int zoneId;         ///< IAS zone id assigned on enrollment (-1 if none).
    bool configured;    ///< True once bind + CIE + activation have been sent.
    double lastPanicTime; ///< Timestamp of the last 0x0500 panic alarm (for debouncing toggles).
} ONICS_BUTTON_T;

#define MAX_ONICS_BUTTONS 32 ///< Maximum Onics buttons tracked.

extern ONICS_BUTTON_T g_onicsButtons[MAX_ONICS_BUTTONS]; ///< Onics registry.
extern int g_numOnicsButtons;   ///< Valid entries in @ref g_onicsButtons.

/// @brief  Initialize the Onics registry and inbox. Call once at startup.
/// @return None.
void OnicsButton_Init( void );

/// @brief  Spawn the Onics worker thread.
/// @return None.
void OnicsButton_Start( void );

/// @brief  Queue an ASSIGN work item so the thread runs setup for a device.
/// @param  shortAddr_  Device network address.
/// @return None.
void OnicsButton_PostAssign( uint16_t shortAddr_ );

/// @brief  Queue an AF message for the Onics thread to parse.
/// @param  shortAddr_  Device network address.
/// @param  af_         The decoded AF message.
/// @return None.
void OnicsButton_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ );

///
/// @brief  Register (or refresh) an Onics button (dispatcher context).
/// @param  shortAddr_  Device network address.
/// @param  endpoint_   Endpoint hosting On/Off + IAS Zone.
/// @return None.
///
void OnicsButton_Discover( uint16_t shortAddr_, uint8_t endpoint_ );

///
/// @brief  Record a resolved IEEE and collapse stale duplicate entries.
/// @param  shortAddr_  Device network address.
/// @param  ieee_       Resolved 8-byte IEEE address.
/// @return None.
///
void OnicsButton_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ );

///
/// @brief  Per-device setup (Onics thread): bind On/Off, write CIE, activate panic.
/// @param  shortAddr_  Device network address. Idempotent via the configured flag.
/// @return None.
///
void OnicsButton_Setup( uint16_t shortAddr_ );

///
/// @brief  Assign a zone id and reply to an IAS Zone enroll request.
/// @param  shortAddr_  Device network address.
/// @param  endpoint_   Source endpoint.
/// @param  transSeq_   ZCL transaction sequence number to echo.
/// @param  zoneType_   Reported IAS zone type.
/// @return None.
///
void OnicsButton_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ );

///
/// @brief  Map a zone status change to a panic set/clear use-case event.
/// @param  shortAddr_   Device network address.
/// @param  zoneStatus_  IAS zone status bitmap (alarm bits in 0x0003).
/// @param  zoneId_      Reported zone id.
/// @return None.
///
void OnicsButton_HandleStatus( uint16_t shortAddr_, uint16_t zoneStatus_, uint8_t zoneId_ );

/// @brief  Print the registered Onics buttons (CLI 'status').
/// @return None.
void OnicsButton_PrintStatus( void );

/// @brief  Is @p shortAddr_ a known Onics button? Thread-safe.
/// @param  shortAddr_  Device network address.
/// @return true if registered.
bool OnicsButton_IsKnown( uint16_t shortAddr_ );

/// @brief  Update a device's last-seen timestamp.
/// @param  shortAddr_  Device network address.
/// @return None.
void OnicsButton_UpdateSeen( uint16_t shortAddr_ );

/// @brief  Re-enumerate endpoints of all known Onics buttons (CLI 'discover').
/// @return None.
void OnicsButton_DiscoverAllActiveEp( void );

/// @brief  Send read requests for Battery and Temperature.
/// @param  shortAddr_  Device network address.
/// @return None.
void OnicsButton_ReadEnvironment( uint16_t shortAddr_ );

#endif // ONICS_BUTTON_H
