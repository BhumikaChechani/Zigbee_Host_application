///
/// @file   sensor_common.h
/// @brief  Message shapes shared between the dispatcher and the sensor threads.
///
/// The dispatcher (main loop) never runs sensor logic itself. Instead it packs
/// a parsed AF message or an assignment notification into a ::SENSOR_MSG_T and
/// posts it to the owning sensor's inbox. Each sensor thread then does its own
/// parsing / device I/O off the critical path.
///
#ifndef SENSOR_COMMON_H
#define SENSOR_COMMON_H

#include <stdint.h>

/// @brief A single decoded AF (Application Framework) incoming message.
typedef struct
{
    uint16_t clusterId;   ///< ZCL cluster id (e.g. 0x0006 On/Off, 0x0500 IAS Zone).
    uint16_t srcAddr;     ///< 16-bit network address of the sending device.
    uint8_t srcEp;        ///< Source endpoint on the sending device.
    uint8_t transSeq;     ///< AF/APS transaction sequence (ZCL TSN is inside @ref data).
    uint8_t dataLen;      ///< Number of valid bytes in @ref data.
    uint8_t data[256];    ///< Raw ZCL payload (frame control, TSN, command, args).
} AF_MSG_T;

/// @brief Discriminator for the work item handed to a sensor thread.
typedef enum
{
    SENSOR_MSG_ASSIGN,    ///< Device was assigned to this module; run its setup.
    SENSOR_MSG_AF         ///< An AF message arrived from one of this module's devices.
} SENSOR_MSG_KIND_T;

/// @brief Work item posted to a sensor worker thread's inbox.
typedef struct
{
    SENSOR_MSG_KIND_T kind; ///< Selects which of the fields below is meaningful.
    uint16_t shortAddr;     ///< Device network address the work item concerns.
    AF_MSG_T af;            ///< Valid only when @ref kind == ::SENSOR_MSG_AF.
} SENSOR_MSG_T;

#endif // SENSOR_COMMON_H
