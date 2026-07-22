///
/// @file   usecase.c
/// @brief  Implementation of the policy layer (see usecase.h).
///
#include "usecase.h"
#include "config.h"
#include "msg_queue.h"

#if ENABLE_AQARA_BUTTON
#include "aqara_button.h"
#endif

#if ENABLE_AQARA_OCCUPANCY
#include "aqara_occupancy.h"
#endif

#if ENABLE_SIREN
#include "siren.h"
#endif

#if ENABLE_CONTACT_SENSOR
#include "contact_sensor.h"
#endif

#include "znp_host.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

// A person walking through the door passes closest to the radar first (often
// below the zone's minimum distance) and only enters the configured zone band
// after the door has already swung shut - so "door open AND presence in zone"
// rarely overlap instantaneously. Treat a door as open for a grace window
// after its CLOSED->OPEN transition so the intrusion condition becomes
// "presence detected AND the door opened recently".
#define DOOR_OPEN_GRACE_S 10.0

static bool ContactCountsAsOpen(int idx, double now) {
    if (g_contactSensors[idx].isOpen)
        return true;
    return g_contactSensors[idx].lastOpenedTime > 0.0 &&
           (now - g_contactSensors[idx].lastOpenedTime) < DOOR_OPEN_GRACE_S;
}

static bool IsDoorOpenForZone(uint8_t zoneIdx) {
    // NOTE: the contact sensor's zoneId is its IAS enrollment id (handed out
    // by g_nextZoneId++, shared with sirens/buttons) - it is NOT related to
    // the FP300's software zone index (0-3). Matching the two only makes
    // sense if the installer deliberately assigned matching ids; in every
    // other case fall back to "is ANY door open", which is the correct
    // semantic for the door+presence alarm.
    bool open = false;
    bool matchedById = false;
    double now = ZNP_GetCurrentTime();

    pthread_mutex_lock(&g_deviceMutex);

    // 1. Exact zoneId match (deliberate multi-door mapping).
    for (int i = 0; i < g_numContactSensors; i++) {
        if (g_contactSensors[i].zoneId == (int)zoneIdx) {
            open = ContactCountsAsOpen(i, now);
            matchedById = true;
            break;
        }
    }

    // 2. Fallback: any registered door open counts. Robust against the IAS
    //    id / zone index mismatch and against stale duplicate entries.
    if (!matchedById) {
        for (int i = 0; i < g_numContactSensors; i++) {
            if (ContactCountsAsOpen(i, now)) {
                open = true;
                break;
            }
        }
    }

    pthread_mutex_unlock(&g_deviceMutex);
    return open;
}

static MSG_QUEUE_T s_useCaseInbox; ///< Inbox of pending use-case events.
static pthread_t s_useCaseThread;  ///< The use-case worker thread handle.

// Maximum allowed gap between consecutive presses in a 3-press sequence.
// If the time since the last recorded press exceeds this, the history is reset.
// The sequence must be: Press 1 -> Press 2 -> Press 3, all within 3.0 seconds
// (measured from Press 1 to Press 3). If the user exceeds this total window,
// the sequence is ignored. This allows normal single presses without triggering.
#define MAX_PRESS_INTERVAL_S 5.0

// Struct to keep track of the last 3 button press timestamps for each Aqara
// switch
typedef struct {
  uint16_t shortAddr;
  double pressTimes[3];
  int count;
  double lastEventTime;
} AqaraPressHistory;

#define MAX_AQARA_HISTORY 32
static AqaraPressHistory s_aqaraHistory[MAX_AQARA_HISTORY];
static int s_numAqaraHistory = 0;

// Helper to find or create a press history entry for a given short address
static AqaraPressHistory *GetAqaraHistory(uint16_t shortAddr) {
  for (int i = 0; i < s_numAqaraHistory; i++) {
    if (s_aqaraHistory[i].shortAddr == shortAddr) {
      return &s_aqaraHistory[i];
    }
  }
  if (s_numAqaraHistory < MAX_AQARA_HISTORY) {
    s_aqaraHistory[s_numAqaraHistory].shortAddr = shortAddr;
    s_aqaraHistory[s_numAqaraHistory].count = 0;
    s_aqaraHistory[s_numAqaraHistory].lastEventTime = 0.0;
    return &s_aqaraHistory[s_numAqaraHistory++];
  }
  return NULL;
}

///
/// @brief  Apply one sensor event to the system (the sole policy mapping
/// point).
///
/// This is the single place that decides what a sensor event does. As new
/// sensors or policies arrive, extend this switch - the sensor modules stay
/// unchanged.
///
/// @param  event_  The event to act on.
/// @return None.
///
static void UseCase_Handle(const UC_EVT_T *event_) {
#if ENABLE_AQARA_BUTTON
  if (AqaraButton_IsKnown(event_->srcAddr) &&
      (event_->type == UC_BUTTON_ON || event_->type == UC_BUTTON_OFF ||
       event_->type == UC_BUTTON_TOGGLE)) {
    double now = ZNP_GetCurrentTime();
    AqaraPressHistory *history = GetAqaraHistory(event_->srcAddr);
    if (history != NULL) {
      double elapsed = now - history->lastEventTime;
      if (elapsed < 0.15) {
        // Hardware debounce: Aqara buttons can fire duplicate ZCL ON/OFF
        // events within ~100ms for a single physical press. Reject anything
        // faster than 150ms to avoid double-counting one press as two.
        LOG_DEBUG("[USECASE] Debouncing Aqara 0x%04X press (elapsed = %.3fs < 0.15s) -> "
               "ignoring duplicate\n",
               event_->srcAddr, elapsed);
        return;
      }
      history->lastEventTime = now;

      if (g_sirenActive) {
        LOG_EVENT("AQARA BTN", event_->srcAddr, "Press Count: 1 (Siren STOP)\n");
#if ENABLE_SIREN
        Siren_ControlAll(0);
#endif
        history->count = 0;
      } else {
        // If the gap from the last recorded press is too long, this is the
        // start of a fresh sequence — discard stale history.
        if (history->count > 0) {
          double gapSinceLastPress = now - history->pressTimes[history->count - 1];
          if (gapSinceLastPress > MAX_PRESS_INTERVAL_S) {
            LOG_DEBUG("[USECASE] Aqara 0x%04X: gap since last press = %.3fs > %.1fs -> "
                   "clearing press history\n",
                   event_->srcAddr, gapSinceLastPress, MAX_PRESS_INTERVAL_S);
            history->count = 0;
          }
        }

        // Record this press (max 3 slots; reset after evaluation at count=3).
        // Press 1: Starts the sequence window.
        // Press 2: Second step within MAX_PRESS_INTERVAL_S of Press 1.
        // Press 3: Final step. If total time (Press3 - Press1) <= 3.0s -> ALARM.
        if (history->count < 3) {
          history->pressTimes[history->count] = now;
          history->count++;
        }

        // Log each intermediate press count so the user can see the sequence
        // building in the console (e.g., count=1, count=2, count=3).
        LOG_EVENT("AQARA BTN", event_->srcAddr, "Press Count: %d\n", history->count);
        LOG_DEBUG("[USECASE] Aqara 0x%04X pressed (count=%d)\n", event_->srcAddr,
               history->count);
        for (int i = 0; i < history->count; i++) {
          LOG_DEBUG("- Press %d: %.3f\n", i + 1, history->pressTimes[i]);
        }

        if (history->count == 3) {
          double diff = history->pressTimes[2] - history->pressTimes[0];
          LOG_DEBUG("[USECASE] Aqara 0x%04X: 3 presses in %.3fs\n",
                 event_->srcAddr, diff);
          // Always reset after a complete 3-press window so the next
          // sequence starts fresh — regardless of whether the window
          // was fast enough to trigger.
          history->count = 0;
          if (diff <= 3.0) {
            LOG_EVENT("AQARA BTN", event_->srcAddr, "3-Press Sequence COMPLETE (%.2fs) -> Siren ON\n", diff);
#if ENABLE_SIREN
            Siren_TriggerAll(1, 240);
#endif
          } else {
            LOG_EVENT("AQARA BTN", event_->srcAddr, "3-Press Sequence TIMEOUT (%.2fs > 3.0s) -> Ignored\n", diff);
          }
        }
      }
    }
    return;
  }
#endif

  switch (event_->type) {
  case UC_BUTTON_ON:
    LOG_EVENT("GENERIC BTN", event_->srcAddr, "Press START\n");
#if ENABLE_SIREN
    Siren_ControlAll(1);
#endif
    break;
  case UC_BUTTON_OFF:
    LOG_EVENT("GENERIC BTN", event_->srcAddr, "Press STOP\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_BUTTON_TOGGLE:
    LOG_DEBUG("[USECASE] TOGGLE from 0x%04X -> sirens %s\n", event_->srcAddr,
           g_sirenActive ? "OFF" : "ON");
#if ENABLE_SIREN
    Siren_ControlAll(g_sirenActive ? 0 : 1);
#endif
    break;
  case UC_PANIC_SET:
    LOG_EVENT("ONICS BTN", event_->srcAddr, "Press START\n");
#if ENABLE_SIREN
    Siren_TriggerAll(6, 240);
#endif
    break;
  case UC_PANIC_CLEAR:
    LOG_EVENT("ONICS BTN", event_->srcAddr, "Press STOP\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_OCCUPANCY_DETECTED: {
#if ENABLE_AQARA_OCCUPANCY
    uint8_t zoneIdx = (uint8_t)event_->raw;
#if ENABLE_CONTACT_SENSOR
    // If a door's last status update is old, a change notification may have
    // been lost - queue an active re-read so the stored state self-heals for
    // the next decision (non-blocking, rate-limited).
    ContactSensor_RefreshIfStale(30.0);
#endif
    bool isDoorOpen = IsDoorOpenForZone(zoneIdx);
    LOG_DEBUG("[USECASE] Person detected in FP300 0x%04X Zone %u (Door open: %s)\n",
           event_->srcAddr, zoneIdx, isDoorOpen ? "YES" : "NO");
    if (isDoorOpen) {
        LOG_EVENT("OCCUPANCY", event_->srcAddr, "Presence DETECTED in Zone %u (%u cm)\n", zoneIdx, event_->val2);
#if ENABLE_SIREN
        Siren_TriggerAll(5, 5); // Mode 5, 5 seconds
#endif
    } else {
        LOG_EVENT("OCCUPANCY", event_->srcAddr, "Presence IGNORED in Zone %u (%u cm) -> Reason: Door is CLOSED\n", zoneIdx, event_->val2);
    }
#else
    LOG_DEBUG("[USECASE] Person detected in zone 0x%04X (index %u)\n",
        event_->srcAddr, event_->raw);
#endif
    break;
  }
  case UC_OCCUPANCY_CLEARED: {
    uint8_t zoneIdx = (uint8_t)event_->raw;
    LOG_EVENT("OCCUPANCY", event_->srcAddr, "Presence CLEARED in Zone %u\n", zoneIdx);
    break;
  }
  case UC_LIGHT_ON:
    LOG_EVENT("OCCUPANCY", event_->srcAddr, "Light ON (Intensity %u)\n", event_->raw);
#if ENABLE_SIREN
    Siren_TriggerAll(6, 240);
#endif
    break;
  case UC_LIGHT_OFF:
    LOG_EVENT("OCCUPANCY", event_->srcAddr, "Light OFF (Intensity %u)\n", event_->raw);
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_CONTACT_OPEN: {
    LOG_EVENT("CONTACT", event_->srcAddr, "Door OPEN\n");
#if ENABLE_SIREN
    Siren_PostBeep(2);
#endif
    break;
  }
  case UC_CONTACT_CLOSED: {
    LOG_EVENT("CONTACT", event_->srcAddr, "Door CLOSED\n");
#if ENABLE_SIREN
    Siren_PostBeep(1);
#endif
    break;
  }
  case UC_VIBRATION_DETECTED:
    LOG_EVENT("VIBRATION", event_->srcAddr, "Vibration DETECTED\n");
#if ENABLE_SIREN
    Siren_TriggerAll(4, 240);
#endif
    break;
  case UC_VIBRATION_CLEARED:
    LOG_EVENT("VIBRATION", event_->srcAddr, "Vibration CLEARED\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_MOVEMENT_DETECTED:
    LOG_EVENT("VIBRATION", event_->srcAddr, "Movement DETECTED\n");
#if ENABLE_SIREN
    Siren_TriggerAll(4, 240);
#endif
    break;
  case UC_MOVEMENT_CLEARED:
    LOG_EVENT("VIBRATION", event_->srcAddr, "Movement CLEARED\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_TAMPER_DETECTED:
    LOG_EVENT("SECURITY", event_->srcAddr, "\033[1;31m(%s) TAMPER DETECTED (Cover Opened)\033[0m\n", Device_GetName(event_->srcAddr));
    break;
  case UC_TAMPER_CLEARED:
    LOG_EVENT("SECURITY", event_->srcAddr, "(%s) Tamper CLEARED (Cover Closed)\n", Device_GetName(event_->srcAddr));
    break;
  case UC_DEVICE_OFFLINE:
    LOG_EVENT("HEALTH", event_->srcAddr, "\033[1;31m(%s) Device OFFLINE (Unreachable)\033[0m\n", Device_GetName(event_->srcAddr));
    break;
  case UC_DEVICE_ONLINE:
    LOG_EVENT("HEALTH", event_->srcAddr, "\033[1;32m(%s) Device Reconnected (ONLINE)\033[0m\n", Device_GetName(event_->srcAddr));
    break;
  default:
    break;
  }
}

///
/// @brief  Use-case worker thread: block on the inbox and act on each event.
/// @param  arg_  Unused thread argument.
/// @return NULL (runs until process exit).
///
static void *UseCase_Thread(void *arg_) {
  (void)arg_;
  while (1) {
    UC_EVT_T *event = (UC_EVT_T *)MsgQueue_Pop(&s_useCaseInbox);
    if (event != NULL) {
      UseCase_Handle(event);
      free(event);
    }
  }
  return NULL;
}

void UseCase_Init(void) { MsgQueue_Init(&s_useCaseInbox); }

void UseCase_Start(void) {
  pthread_create(&s_useCaseThread, NULL, UseCase_Thread, NULL);
}

void UseCase_Post(UC_EVT_TYPE_T type_, uint16_t srcAddr_, uint16_t raw_, uint32_t val2_) {
  UC_EVT_T *event = (UC_EVT_T *)malloc(sizeof(UC_EVT_T));
  if (event == NULL) {
    return;
  }
  event->type = type_;
  event->srcAddr = srcAddr_;
  event->raw = raw_;
  event->val2 = val2_;
  MsgQueue_Push(&s_useCaseInbox, event);
}
