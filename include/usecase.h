///
/// @file   usecase.h
/// @brief  Use-case (policy) layer: turns sensor events into system behavior.
///
/// Sensor threads translate raw device traffic into high-level events and post
/// them here; a dedicated use-case thread collects them and decides what the
/// system does (drive the sirens). This is the single
/// place to change behavior as new sensors are added - sensor modules stay
/// device-focused and never call the siren directly.
///
#ifndef USECASE_H
#define USECASE_H

#include <stdint.h>

/// @brief High-level events a sensor can raise for the policy layer to act on.
typedef enum
{
    UC_BUTTON_ON,       ///< A button/switch requested "on"  -> sirens on.
    UC_BUTTON_OFF,      ///< A button/switch requested "off" -> sirens off.
    UC_BUTTON_TOGGLE,   ///< A button/switch toggle          -> flip siren state.
    UC_PANIC_SET,       ///< A panic alarm became active     -> sirens on.
    UC_PANIC_CLEAR,     ///< A panic alarm cleared           -> sirens off.
    UC_OCCUPANCY_DETECTED,///< Occupancy detected            -> sirens on.
    UC_OCCUPANCY_CLEARED, ///< Occupancy cleared             -> sirens off.
    UC_LIGHT_ON,          ///< Light turned on               -> sirens off.
    UC_LIGHT_OFF,         ///< Light turned off              -> sirens on.
    UC_CONTACT_OPEN,      ///< Contact sensor opened         -> sirens on.
    UC_CONTACT_CLOSED,    ///< Contact sensor closed         -> sirens off.
    UC_VIBRATION_DETECTED,///< Vibration detected            -> sirens on.
    UC_VIBRATION_CLEARED, ///< Vibration cleared             -> sirens off.
    UC_MOVEMENT_DETECTED, ///< Movement/Tilt detected        -> sirens on.
    UC_MOVEMENT_CLEARED,  ///< Movement/Tilt cleared         -> sirens off.
    UC_TAMPER_DETECTED,   ///< Siren physical tamper         -> sirens on.
    UC_TAMPER_CLEARED,    ///< Siren physical tamper cleared -> sirens off.
    UC_DEVICE_OFFLINE,    ///< Device stopped reporting      -> log warning
    UC_DEVICE_ONLINE      ///< Device resumed reporting      -> log info
} UC_EVT_TYPE_T;

/// @brief One event enqueued to the use-case thread.
typedef struct
{
    UC_EVT_TYPE_T type; ///< What happened.
    uint16_t srcAddr;   ///< Device that raised it (for logging/traceability).
    uint16_t raw;       ///< Raw cmd id / zone status, for logging only.
    uint32_t val2;      ///< Extra data like distance or intensity.
} UC_EVT_T;

///
/// @brief  Initialize the use-case inbox. Call once before UseCase_Start().
/// @return None.
///
void UseCase_Init( void );

///
/// @brief  Spawn the use-case worker thread that consumes and acts on events.
/// @return None.
///
void UseCase_Start( void );

///
/// @brief  Post a high-level event to the use-case thread (non-blocking).
///
/// Called from any sensor thread. The event is queued and handled
/// asynchronously by the use-case thread, so the caller never blocks on the
/// resulting siren I/O.
///
/// @param  type_     Event kind (see ::UC_EVT_TYPE_T).
/// @param  srcAddr_  Network address of the device that raised the event.
/// @param  raw_      Raw command id / zone status, for logging.
/// @return None.
///
void UseCase_Post( UC_EVT_TYPE_T type_, uint16_t srcAddr_, uint16_t raw_, uint32_t val2_ );

#endif // USECASE_H
