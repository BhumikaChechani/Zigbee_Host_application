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

#include "znp_host.h"
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

static MSG_QUEUE_T s_useCaseInbox; ///< Inbox of pending use-case events.
static pthread_t s_useCaseThread;  ///< The use-case worker thread handle.

// Maximum allowed gap between consecutive presses in a 3-press sequence.
// If the time since the last recorded press exceeds this, the history is reset.
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
      if (elapsed < 0.10) {
        printf("[USECASE] Debouncing Aqara 0x%04X press (elapsed = %.3fs) -> "
               "ignoring\n",
               event_->srcAddr, elapsed);
        return;
      }
      history->lastEventTime = now;

      if (g_sirenActive) {
        printf("[USECASE] Aqara 0x%04X pressed while siren is active -> "
               "turning sirens OFF\n",
               event_->srcAddr);
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
            printf("[USECASE] Aqara 0x%04X: gap since last press = %.3fs > %.1fs -> "
                   "clearing press history\n",
                   event_->srcAddr, gapSinceLastPress, MAX_PRESS_INTERVAL_S);
            history->count = 0;
          }
        }

        // Record this press (max 3 slots; do NOT slide — reset after evaluation)
        if (history->count < 3) {
          history->pressTimes[history->count] = now;
          history->count++;
        }

        printf("[USECASE] Aqara 0x%04X pressed (count=%d)\n", event_->srcAddr,
               history->count);
        for (int i = 0; i < history->count; i++) {
          printf("  - Press %d: %.3f\n", i + 1, history->pressTimes[i]);
        }

        if (history->count == 3) {
          double diff = history->pressTimes[2] - history->pressTimes[0];
          printf("[USECASE] Aqara 0x%04X: 3 presses in %.3fs\n",
                 event_->srcAddr, diff);
          // Always reset after a complete 3-press window so the next
          // sequence starts fresh — regardless of whether the window
          // was fast enough to trigger.
          history->count = 0;
          if (diff <= 3.0) {
            printf("[USECASE] 3 presses in <= 3.0s -> turning sirens ON (Burglar / FULL CAPACITY)\n");
#if ENABLE_SIREN
            Siren_SetMode(1);   // Mode 1 = Burglar
            Siren_SetVolume(3); // Ensure max capacity
            Siren_ControlAll(1);
#endif
          } else {
            printf("[USECASE] 3 presses but window too wide (%.3fs > 3.0s) -> "
                   "ignoring\n", diff);
          }
        }
      }
    }
    return;
  }
#endif

  switch (event_->type) {
  case UC_BUTTON_ON:
    printf("[USECASE] ON from 0x%04X -> sirens ON\n", event_->srcAddr);
#if ENABLE_SIREN
    Siren_ControlAll(1);
#endif
    break;
  case UC_BUTTON_OFF:
    printf("[USECASE] OFF from 0x%04X -> sirens OFF\n", event_->srcAddr);
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_BUTTON_TOGGLE:
    printf("[USECASE] TOGGLE from 0x%04X -> sirens %s\n", event_->srcAddr,
           g_sirenActive ? "OFF" : "ON");
#if ENABLE_SIREN
    Siren_ControlAll(g_sirenActive ? 0 : 1);
#endif
    break;
  case UC_PANIC_SET:
    printf("[USECASE] PANIC (0x%04X, status=0x%04X) -> sirens ON\n",
           event_->srcAddr, event_->raw);
#if ENABLE_SIREN
    Siren_ControlAll(1);
#endif
    break;
  case UC_PANIC_CLEAR:
    printf("[USECASE] panic cleared (0x%04X) -> sirens OFF\n", event_->srcAddr);
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_OCCUPANCY_DETECTED: {
#if ENABLE_AQARA_OCCUPANCY
    // Always log any-zone detection, then alarm only when EVERY
    // registered occupancy zone reports presence.
    int occupied = AqaraOccupancy_OccupiedCount();
    int total = g_numAqaraOccupancies;
    printf(
        "🚶 [USECASE] Person detected in zone 0x%04X (index %u) (%d/%d physical sensors occupied)\n",
        event_->srcAddr, event_->raw, occupied, total);
    if (AqaraOccupancy_AllOccupied()) {
      printf("[USECASE] OCCUPANCY in ALL physical sensors -> sirens ON\n");
      // #if ENABLE_SIREN
      // Siren_ControlAll( 1 );
      // #endif
    } else {
      printf("[USECASE] Not all physical sensors occupied -> holding\n");
    }
#else
    printf(
        "🚶 [USECASE] Person detected in zone 0x%04X (index %u)\n",
        event_->srcAddr, event_->raw);
#endif
    break;
  }
  case UC_OCCUPANCY_CLEARED:
    printf("💨 [USECASE] Occupancy cleared in zone 0x%04X (index %u) -> sirens OFF\n",
           event_->srcAddr, event_->raw);
    // Siren_ControlAll( 0 );
    break;
  case UC_LIGHT_ON:
    printf("☀️ [USECASE] Light turned ON -> sirens ON (Emergency Panic)\n");
#if ENABLE_SIREN
    Siren_SetMode(6);
    Siren_ControlAll(1);
#endif
    break;
  case UC_LIGHT_OFF:
    printf("🌙 [USECASE] Light turned OFF -> sirens OFF\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_CONTACT_OPEN:
    printf("🚪 [USECASE] Contact Sensor OPENED -> Siren Beep 2 times\n");
#if ENABLE_SIREN
    Siren_Beep(2);
#endif
    break;
  case UC_CONTACT_CLOSED:
    printf("🚪 [USECASE] Contact Sensor CLOSED -> Siren Beep 1 time\n");
#if ENABLE_SIREN
    Siren_Beep(1);
#endif
    break;
  case UC_VIBRATION_DETECTED:
    printf("🔴 📳 [USECASE] Vibration Sensor (Alarm 2) ALARM -> sirens ON (Police Panic)\n");
#if ENABLE_SIREN
    Siren_SetMode(4);
    Siren_ControlAll(1);
#endif
    break;
  case UC_VIBRATION_CLEARED:
    printf("🟢 📴 [USECASE] Vibration Sensor (Alarm 2) CLEARED -> sirens OFF\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_MOVEMENT_DETECTED:
    printf("🔴 🫨 [USECASE] Movement/Tilt Sensor (Alarm 1) ALARM -> sirens ON (Police Panic)\n");
#if ENABLE_SIREN
    Siren_SetMode(4);
    Siren_ControlAll(1);
#endif
    break;
  case UC_MOVEMENT_CLEARED:
    printf("🟢 🧍 [USECASE] Movement/Tilt Sensor (Alarm 1) CLEARED -> sirens OFF\n");
#if ENABLE_SIREN
    Siren_ControlAll(0);
#endif
    break;
  case UC_TAMPER_DETECTED:
    printf("🚨 [USECASE] Siren TAMPER Switch OPENED -> sirens ON (DISABLED FOR NOW)\n");
    break;
  case UC_TAMPER_CLEARED:
    printf("✅ [USECASE] Siren TAMPER Switch CLOSED -> sirens OFF (DISABLED FOR NOW)\n");
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

void UseCase_Post(UC_EVT_TYPE_T type_, uint16_t srcAddr_, uint16_t raw_) {
  UC_EVT_T *event = (UC_EVT_T *)malloc(sizeof(UC_EVT_T));
  if (event == NULL) {
    return;
  }
  event->type = type_;
  event->srcAddr = srcAddr_;
  event->raw = raw_;
  MsgQueue_Push(&s_useCaseInbox, event);
}
