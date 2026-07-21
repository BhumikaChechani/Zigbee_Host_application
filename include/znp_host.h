///
/// @file   znp_host.h
/// @brief  Serial transport, MT framing, and typed ZNP command wrappers.
///
/// This module owns the serial link to the CC1352P7 running as a pure ZNP. It
/// runs a background reader thread that frames the MT byte stream, routes SRSP
/// responses to the blocking ZNP_Sreq() caller, and pushes asynchronous AREQ
/// indications onto ::g_eventQueue for the dispatcher. Every higher-level MT
/// command (SYS/UTIL/AF/ZDO) is exposed here as a typed, thread-safe wrapper.
///
#ifndef ZNP_HOST_H
#define ZNP_HOST_H

#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define LOG_LEVEL_DEBUG 0
#define LOG_LEVEL_INFO  1

extern int g_logLevel;

#define LOG_DEBUG(...) do { if (g_logLevel <= LOG_LEVEL_DEBUG) { printf(__VA_ARGS__); } } while(0)
#define LOG_INFO(...)  do { if (g_logLevel <= LOG_LEVEL_INFO)  { printf(__VA_ARGS__); } } while(0)

static inline void print_timestamp(void) {
    time_t now;
    time(&now);
    struct tm *local = localtime(&now);
    printf("[%02d:%02d:%02d] ", local->tm_hour, local->tm_min, local->tm_sec);
}

#define LOG_EVENT(MODULE, ADDR, ...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_INFO) { \
            print_timestamp(); \
            printf("[INFO] [%-10s] [0x%04X] ", MODULE, ADDR); \
            printf(__VA_ARGS__); \
        } \
    } while(0)


#define PORT_DEFAULT "/dev/ttyACM0" ///< Default serial device if none is given.
#define PERMIT_JOIN_DURATION 0xFE   ///< Permit-join window, 254 seconds.
#define PERMIT_JOIN_REFRESH 180     ///< Re-open permit join every 3 minutes.

// ZNP NV configuration item IDs (1:1 with the NV item IDs used by SYS_OSAL_NV).
#define ZCD_NV_STARTUP_OPTION                                                  \
  0x03                           ///< Startup option (CLEAR_CONFIG/CLEAR_STATE).
#define ZCD_NV_LOGICAL_TYPE 0x87 ///< Device logical type (0 = coordinator).
#define ZCD_NV_PANID 0x83        ///< Personal Area Network id.
#define ZCD_NV_CHANLIST 0x84     ///< Channel bitmask.
#define ZCD_NV_PRECFGKEY 0x62    ///< Pre-configured Trust Center link key.
#define ZCD_NV_PRECFGKEYS_ENABLE                                               \
  0x63 ///< Whether the pre-configured key is distributed.
#define ZCD_NV_ZDO_DIRECT_CB                                                   \
  0x8F ///< Deliver ZDO callbacks directly to the host.

#define STARTOPT_CLEAR_CONFIG                                                  \
  0x01 ///< Startup option bit: wipe configuration NV.
#define STARTOPT_CLEAR_STATE                                                   \
  0x02 ///< Startup option bit: wipe network state NV.

#define NETWORK_PAN_ID 0x1A2B ///< PAN id this coordinator forms.
#define NETWORK_CHANNEL 15    ///< 802.15.4 channel this coordinator uses.
#define NETWORK_TC_LINK_KEY_HEX                                                \
  "5a6967426565416c6c69616e63653039" ///< "ZigBeeAlliance09".

/// @brief One decoded MT frame (length, command bytes, and payload).
typedef struct {
  uint8_t len;          ///< Payload length in bytes.
  uint8_t cmd0;         ///< MT command byte 0 (type + subsystem).
  uint8_t cmd1;         ///< MT command byte 1 (command id).
  uint8_t payload[256]; ///< Frame payload.
} MT_FRAME_T;

#define EVENT_QUEUE_MAX 256 ///< Capacity of the AREQ event ring buffer.

/// @brief Thread-safe ring buffer of asynchronous AREQ frames from the ZNP.
typedef struct {
  MT_FRAME_T frames[EVENT_QUEUE_MAX]; ///< Ring storage.
  int head;                           ///< Index of the oldest frame.
  int tail;              ///< Index where the next frame is written.
  int count;             ///< Number of frames currently buffered.
  pthread_mutex_t mutex; ///< Guards the ring state.
  pthread_cond_t cond;   ///< Signalled when a frame is pushed.
} EVENT_QUEUE_T;

/// @brief Recursive mutex protecting the global device registries.
extern pthread_mutex_t g_deviceMutex;

#define MAX_DISCOVERED_IEEES 128 ///< Max short-address -> IEEE mappings cached.

/// @brief One short-address to IEEE (extended address) mapping.
typedef struct {
  uint16_t shortAddr; ///< 16-bit network address.
  uint8_t ieee[8];    ///< 64-bit IEEE address (little-endian).
} IEEE_MAPPING_T;

extern IEEE_MAPPING_T
    g_discoveredIeees[MAX_DISCOVERED_IEEES]; ///< Address cache.
extern int g_numDiscoveredIeees; ///< Number of valid entries in @ref
                                 ///< g_discoveredIeees.

extern bool g_sirenActive;   ///< True while at least one siren is sounding.
extern uint8_t g_nextZoneId; ///< Next IAS zone id to hand out on enrollment.
extern uint8_t
    g_coordinatorIeee[8]; ///< This coordinator's IEEE (little-endian).
extern bool
    g_hasCoordinatorIeee;  ///< True once @ref g_coordinatorIeee is known.


extern EVENT_QUEUE_T g_eventQueue; ///< AREQ indications from the reader thread.

// ---------------------------------------------------------------------------
// Thread-safe Event Queue
// ---------------------------------------------------------------------------

/// @brief  Initialize the AREQ event ring buffer.
/// @param  queue_  Queue to initialize.
/// @return None.
void EventQueue_Init(EVENT_QUEUE_T *queue_);

/// @brief  Push a copy of @p frame_ onto the queue; drops it if full.
/// @param  queue_  Target queue.
/// @param  frame_  Frame to copy in.
/// @return None.
void EventQueue_Push(EVENT_QUEUE_T *queue_, const MT_FRAME_T *frame_);

/// @brief  Pop the oldest frame, waiting up to @p timeoutMs_ for one to arrive.
/// @param  queue_     Source queue.
/// @param  frame_     Destination for the dequeued frame.
/// @param  timeoutMs_ Max time to block, in milliseconds.
/// @return true if a frame was returned; false on timeout.
bool EventQueue_Pop(EVENT_QUEUE_T *queue_, MT_FRAME_T *frame_, int timeoutMs_);

/// @brief  Discard all queued frames (used around a chip reset).
/// @param  queue_  Queue to clear.
/// @return None.
void EventQueue_Clear(EVENT_QUEUE_T *queue_);

// ---------------------------------------------------------------------------
// Serial & MT Core
// ---------------------------------------------------------------------------

/// @brief  Open/configure the serial port (115200 8N1 raw) and start the
/// reader.
/// @param  port_  Serial device path (e.g. "/dev/ttyACM0").
/// @return true on success; false if the port cannot be opened/configured.
bool ZNP_Init(const char *port_);

/// @brief  Stop the reader thread and close the serial port.
/// @return None.
void ZNP_Close(void);

/// @brief  Compute the MT frame check sequence (XOR of length, cmds, payload).
/// @param  len_      Payload length.
/// @param  cmd0_     Command byte 0.
/// @param  cmd1_     Command byte 1.
/// @param  payload_  Payload bytes (may be NULL when @p len_ is 0).
/// @return The 1-byte FCS.
uint8_t ZNP_CalcFcs(uint8_t len_, uint8_t cmd0_, uint8_t cmd1_,
                    const uint8_t *payload_);

///
/// @brief  Send a synchronous request (SREQ) and wait for its SRSP.
///
/// Serialized by an internal mutex, so it is safe to call from any thread. The
/// reader thread matches the response by the expected cmd0/cmd1 and hands it
/// back through a condition variable.
///
/// @param  cmd0_       Request command byte 0.
/// @param  cmd1_       Request command byte 1.
/// @param  payload_    Request payload (may be NULL when @p len_ is 0).
/// @param  len_        Payload length.
/// @param  rxFrame_    Out: the received SRSP frame (may be NULL to ignore it).
/// @param  timeoutMs_  Max time to wait for the SRSP, in milliseconds.
/// @return true if a matching SRSP was received; false on write error/timeout.
///
bool ZNP_Sreq(uint8_t cmd0_, uint8_t cmd1_, const uint8_t *payload_,
              uint8_t len_, MT_FRAME_T *rxFrame_, int timeoutMs_);

// ---------------------------------------------------------------------------
// MT Command API Wrappers
// ---------------------------------------------------------------------------

/// @brief  SYS_PING - verify the ZNP is alive and responding.
/// @return true if a valid ping response was received.
bool ZNP_SysPing(void);

///
/// @brief  Reset the ZNP and wait for the SYS_RESET_IND that follows reboot.
/// @param  hard_  true = hardware reset, false = soft reset.
/// @return true if SYS_RESET_IND was observed; false otherwise.
///
bool ZNP_SysResetReq(bool hard_);

///
/// @brief  UTIL_GET_DEVICE_INFO - read IEEE/short addr/type/state and cache
/// IEEE.
/// @return The device logical state (9 = coordinator), or -1 on failure.
///
int ZNP_UtilGetDeviceInfo(void);

///
/// @brief  ZDO_STARTUP_FROM_APP - bring the network up.
/// @param  startDelayMs_  Delay before startup, in milliseconds.
/// @return The start mode (0 = restored, 1 = new, 2 = leave/retry), or -1.
///
int ZNP_ZdoStartupFromApp(uint16_t startDelayMs_);

///
/// @brief  SYS_OSAL_NV_WRITE - write an NV item.
/// @param  nvId_    NV item id.
/// @param  value_   Bytes to write.
/// @param  len_     Number of bytes.
/// @param  offset_  Offset within the item.
/// @return true if the ZNP reported success (status 0).
///
bool ZNP_NvWrite(uint16_t nvId_, const uint8_t *value_, uint8_t len_,
                 uint8_t offset_);

///
/// @brief  SYS_OSAL_NV_READ - read an NV item.
/// @param  nvId_    NV item id.
/// @param  offset_  Offset within the item.
/// @param  rxBuf_   Out: buffer receiving the value (may be NULL).
/// @param  rxLen_   Out: number of bytes read (may be NULL).
/// @return true if the ZNP reported success (status 0).
///
bool ZNP_NvRead(uint16_t nvId_, uint8_t offset_, uint8_t *rxBuf_,
                uint8_t *rxLen_);

///
/// @brief  Write a coordinator config item via NV, then read it back to verify.
///
/// Wraps ZNP_NvWrite()/ZNP_NvRead() and prints the write + read-back result,
/// so a mis-provisioned item is visible immediately. Used by ZNP_FactoryNew().
///
/// @param  cfgId_  ZCD_NV configuration id (see the ZCD_NV_* macros).
/// @param  value_  Bytes to write.
/// @param  len_    Number of bytes.
/// @return true if written and the read-back matches (key item allows
/// write-only).
///
bool ZNP_WriteConfiguration(uint8_t cfgId_, const uint8_t *value_,
                            uint8_t len_);

///
/// @brief  Full factory-new provisioning: wipe NV, then write coordinator
/// config.
///
/// Sets the CLEAR flag and resets so the chip erases NV, then writes logical
/// type, PAN id, channel, TC link key and ZDO_DIRECT_CB, and resets again to
/// apply. Escalates soft->hard reset if a reset is not confirmed.
///
/// @return true when the sequence completes (best-effort; see printed status).
///
bool ZNP_FactoryNew(void);

///
/// @brief  Open the network for joining via several permit-join methods.
/// @param  duration_  Permit-join window in seconds.
/// @return true if at least one ZDO permit-join request succeeded.
///
bool ZNP_PermitJoin(uint8_t duration_);

///
/// @brief  Set the Trust Center "require key exchange" BDB policy.
///
/// Zigbee 3.0 defaults to requiring every joining device to update its Trust
/// Center Link Key shortly after joining; devices that do not (many Aqara
/// sensors) are removed by the TC and end up in an endless rejoin loop. Passing
/// @p require_ = false lets such devices stay on the well-known global key.
/// Uses MT_APP_CNF APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE (0x2F / 0x09).
///
/// @param  require_  true to require the exchange (Z3.0 default), false to allow
///                   devices to remain on the global key.
/// @return true if the ZNP acknowledged the command with status 0.
///
bool ZNP_BdbSetTcRequireKeyExchange(bool require_);

///
/// @brief  AF_REGISTER - register the host application endpoint.
/// @param  endpoint_        Endpoint number to register.
/// @param  profileId_       Application profile id (e.g. 0x0104 HA).
/// @param  deviceId_        Device id within the profile.
/// @param  deviceVersion_   Device version.
/// @param  latency_         Latency request field.
/// @param  numInClusters_   Number of input (server) clusters.
/// @param  inClusters_      Input cluster id array.
/// @param  numOutClusters_  Number of output (client) clusters.
/// @param  outClusters_     Output cluster id array.
/// @return true if the endpoint registered successfully.
///
bool ZNP_AfRegister(uint8_t endpoint_, uint16_t profileId_, uint16_t deviceId_,
                    uint8_t deviceVersion_, uint8_t latency_,
                    uint8_t numInClusters_, const uint16_t *inClusters_,
                    uint8_t numOutClusters_, const uint16_t *outClusters_);

///
/// @brief  Register to receive a ZDO response cluster as an MT callback.
/// @param  clusterId_  ZDO cluster id (e.g. 0x8004 Simple_desc_rsp, 0x0013
/// annce).
/// @return true if registration succeeded.
///
bool ZNP_ZdoMsgCbRegister(uint16_t clusterId_);

///
/// @brief  AF_DATA_REQUEST_EXT - send an application (ZCL) frame to a device.
/// @param  dstAddrMode_  Address mode (2 = 16-bit short, 3 = 64-bit IEEE).
/// @param  dstAddr_      Destination address (short in low bits, or full IEEE).
/// @param  dstEndpoint_  Destination endpoint.
/// @param  panId_        Destination PAN id (0 for the local network).
/// @param  srcEndpoint_  Source (host) endpoint.
/// @param  clusterId_    ZCL cluster id.
/// @param  transId_      AF transaction id.
/// @param  options_      AF options bitmap.
/// @param  radius_       Network radius (hop limit).
/// @param  data_         ZCL payload bytes.
/// @param  dataLen_      Payload length.
/// @return true if the ZNP accepted the request (status 0).
///
bool ZNP_AfDataRequestExt(uint8_t dstAddrMode_, uint64_t dstAddr_,
                          uint8_t dstEndpoint_, uint16_t panId_,
                          uint8_t srcEndpoint_, uint16_t clusterId_,
                          uint8_t transId_, uint8_t options_, uint8_t radius_,
                          const uint8_t *data_, uint16_t dataLen_);

///
/// @brief  Send a ZCL Default Response.
/// @param  shortAddr_    Destination short address.
/// @param  dstEndpoint_  Destination endpoint.
/// @param  srcEndpoint_  Source endpoint on the coordinator (e.g. 1 or 8).


///
/// @brief  ZDO_MATCH_DESC_REQ - find devices matching a cluster profile.
/// @param  shortAddr_       Destination/interest address (e.g. 0xFFFD
/// broadcast).
/// @param  profileId_       Profile id to match.
/// @param  numInClusters_   Number of input clusters to match.
/// @param  inClusters_      Input cluster id array.
/// @param  numOutClusters_  Number of output clusters to match.
/// @param  outClusters_     Output cluster id array.
/// @return true if the request was accepted.
///
bool ZNP_ZdoMatchDescReq(uint16_t shortAddr_, uint16_t profileId_,
                         uint8_t numInClusters_, const uint16_t *inClusters_,
                         uint8_t numOutClusters_, const uint16_t *outClusters_);

///
/// @brief  ZDO_ACTIVE_EP_REQ - ask a device to list its active endpoints.
/// @param  shortAddr_  Target device network address.
/// @return true if the request was accepted.
///
bool ZNP_ZdoActiveEpReq(uint16_t shortAddr_);

///
/// @brief  ZDO_MGMT_LEAVE_REQ - tell a device to leave the network.
/// @param  shortAddr_      Target device network address.
/// @param  extAddr_        Optional IEEE address of the device to remove (NULL to use shortAddr only).
/// @param  removeChildren_ True to remove children as well.
/// @param  rejoin_         True to instruct the device to rejoin immediately.
/// @return true if the request was accepted.
///
bool ZNP_ZdoMgmtLeaveReq(uint16_t shortAddr_, const uint8_t *extAddr_, bool removeChildren_, bool rejoin_);

///
/// @brief  ZDO_SIMPLE_DESC_REQ - fetch one endpoint's descriptor (clusters).
/// @param  shortAddr_  Target device network address.
/// @param  endpoint_   Endpoint to describe.
/// @return true if the request was accepted.
///
bool ZNP_QuerySimpleDesc(uint16_t shortAddr_, uint8_t endpoint_);

///
/// @brief  ZDO_BIND_REQ - bind a device's cluster to the coordinator endpoint.
/// @param  buttonShortAddr_  Device network address to configure.
/// @param  buttonIeee_       Device IEEE address.
/// @param  srcEndpoint_      Device source endpoint being bound.
/// @param  clusterId_        Cluster id to bind.
/// @param  coordIeee_        Coordinator IEEE (bind destination).
/// @param  coordEndpoint_    Coordinator endpoint (bind destination).
/// @return true if the request was accepted.
///
bool ZNP_ZdoBindReq(uint16_t buttonShortAddr_, const uint8_t *buttonIeee_,
                    uint8_t srcEndpoint_, uint16_t clusterId_,
                    const uint8_t *coordIeee_, uint8_t coordEndpoint_);

///
/// @brief  Write the IAS CIE address attribute (0x0010) = this coordinator.
/// @param  buttonShortAddr_  Target IAS Zone device.
/// @param  buttonEndpoint_   Target endpoint hosting the IAS Zone cluster.
/// @param  transId_          ZCL transaction sequence number.
/// @return true if the ZCL write was accepted by the ZNP.
///
bool ZNP_WriteCieAddress(uint16_t buttonShortAddr_, uint8_t buttonEndpoint_,
                         uint8_t transId_);

///
/// @brief  Send the Onics manufacturer-specific "panic activation" write.
/// @param  buttonShortAddr_  Target Onics button.
/// @param  buttonEndpoint_   Target endpoint.
/// @param  transId_          ZCL transaction sequence number.
/// @return true if the ZCL write was accepted by the ZNP.
///
bool ZNP_SendButtonActivation(uint16_t buttonShortAddr_,
                              uint8_t buttonEndpoint_, uint8_t transId_);

///
/// @brief  Reply to an IAS Zone Enroll Request with a success response.
/// @param  buttonShortAddr_  Enrolling device.
/// @param  buttonEndpoint_   Endpoint hosting the IAS Zone cluster.
/// @param  transId_          ZCL transaction sequence number (echo the
/// request).
/// @param  zoneId_           Zone id assigned to the device.
/// @return true if the response was accepted by the ZNP.
///
bool ZNP_SendZoneEnrollResponse(uint16_t buttonShortAddr_,
                                uint8_t buttonEndpoint_, uint8_t transId_,
                                uint8_t zoneId_);

/// @brief  IAS WD Start Warning - start or stop a siren.
/// @param  sirenShortAddr_  Target siren network address.
/// @param  sirenEndpoint_   Endpoint hosting the IAS WD cluster.
/// @param  transId_         ZCL transaction sequence number.
/// @param  warnMode_        Warning mode (0 = stop, non-zero = warn/burglar).
/// @param  volume_          Volume level (0=low, 1=medium, 2=high, 3=very high).
/// @param  duration_        Warning duration in seconds.
/// @return true if the command was accepted by the ZNP.
///
bool ZNP_SendSirenWarning(uint16_t sirenShortAddr_, uint8_t sirenEndpoint_,
                          uint8_t transId_, uint8_t warnMode_, uint8_t volume_,
                          uint16_t duration_);

/// @brief  IAS WD Squawk - send a short chime/beep.
/// @param  sirenShortAddr_  Target siren network address.
/// @param  sirenEndpoint_   Endpoint hosting the IAS WD cluster.
/// @param  transId_         ZCL transaction sequence number.
/// @param  squawkMode_      Squawk mode (e.g. 0=armed).
/// @param  volume_          Squawk volume (0=low, 1=medium, 2=high, 3=very high).
/// @return true if the command was accepted by the ZNP.
bool ZNP_SendSirenSquawk(uint16_t sirenShortAddr_, uint8_t sirenEndpoint_,
                         uint8_t transId_, uint8_t squawkMode_, uint8_t volume_);

/// @brief  Send a ZCL Default Response.
/// @param  shortAddr_   Target network address.
/// @param  endpoint_    Target endpoint.
/// @param  clusterId_   ZCL cluster id.
/// @param  transId_     ZCL transaction sequence number.
/// @param  cmdId_       ZCL command ID being responded to.
/// @param  status_      Status code.
/// @return true if accepted.
bool ZNP_SendDefaultResponse(uint16_t shortAddr_, uint8_t endpoint_, uint16_t clusterId_,
                             uint8_t transId_, uint8_t cmdId_, uint8_t status_);

/// @brief  Monotonic-ish wall-clock time in seconds (for timeouts/last-seen).
/// @return Current time in seconds as a double.
double ZNP_GetCurrentTime(void);

// ---------------------------------------------------------------------------
// Device Registry Helpers (implemented in main.c)
// ---------------------------------------------------------------------------

/// @brief  Cache/refresh a short-address -> IEEE mapping. Thread-safe.
/// @param  shortAddr_  Device network address.
/// @param  ieee_       8-byte IEEE address to store.
/// @return None.
void Device_AddDiscoveredIeee(uint16_t shortAddr_, const uint8_t *ieee_);

/// @brief  Look up a device's cached IEEE address. Thread-safe.
/// @param  shortAddr_  Device network address.
/// @param  ieeeOut_    Out: 8-byte IEEE (may be NULL to test presence only).
/// @return true if a mapping exists.
bool Device_GetDiscoveredIeee(uint16_t shortAddr_, uint8_t *ieeeOut_);

/// @brief  Persist all registered devices to the devices.txt file.
/// @return None.
void Device_Save(void);

/// @brief  Load previously persisted devices from devices.txt at startup.
/// @return None.
void Device_Load(void);

#endif // ZNP_HOST_H
