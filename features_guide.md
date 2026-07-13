# QA Testing Guide: Zigbee Host Application

This document provides a complete overview of the available commands, sensor behaviors, and specific event triggers (including siren logic) to assist the QA team in testing the Zigbee Host Application.

---

## 💻 1. Available CLI Commands

Once the application is running, you can interact with the network via the terminal.

### General & System Management
| Command | Description |
| :--- | :--- |
| **`help`** | Prints a list of all available commands. |
| **`status`** | Displays a summary of all registered devices, their network addresses, last seen times, and active configurations. |
| **`discover [addr hex]`** | Broadcasts discovery requests to find new devices on the network. If a specific address is provided, it actively queries only that device. |
| **`permit [duration]`** | Opens the Zigbee network for new devices to join (defaults to 60 seconds). Example: `permit 120`. |
| **`exit`** / **`quit`** | Safely closes the serial connection and exits the application. |

### Aqara FP300 Presence Sensor
| Command | Description |
| :--- | :--- |
| **`zone <addr> <idx> <start> <end>`** | Configures a detection zone. Distance slices are 25cm each (e.g., `zone 7AF2 0 0 2` covers 0-50cm). |
| **`zonedel <addr> <idx>`** | Deletes a previously configured zone. |
| **`sensitivity <addr> <1\|2\|3>`** | Sets the sensor's motion sensitivity (1 = Low, 2 = Medium, 3 = High). |
| **`spatiallearn <addr>`** | Triggers the sensor's spatial learning calibration (ensure room is empty first). |
| **`env <addr>`** | Requests the sensor to report its current environmental readings (e.g., temperature). |
| **`forcesetup <addr>`** | Forces a full re-initialization and Zigbee binding setup for the sensor. |

### Siren Controls
| Command | Description |
| :--- | :--- |
| **`siren on`** | Turns **all** sirens ON simultaneously. |
| **`siren off`** | Turns **all** sirens OFF simultaneously. |
| **`siren vol <0-3>`** | Sets the global siren volume (0 = Low, 1 = Medium, 2 = High, 3 = Very High). |
| **`siren test <addr> <ep>`** | Sends a test warning to a specific siren endpoint. |

---

## 📡 2. Sensor Behaviors & Event Triggers

This section details exactly what happens in the system when different sensors are triggered. All logic is handled by the `usecase.c` component.

### Aqara Wireless Mini Switch T1
- **Triple Press (within 3 seconds):** Evaluated as an alarm condition. **Action:** Turns the Siren **ON**.
- **Single Press (while Siren is actively ringing):** Acts as a manual override. **Action:** Turns the Siren **OFF**.
- **Any other press sequence:** Logged, but no system action is taken.

### Onics Smart/Panic Button (SBTZB-110)
- **Panic State Triggered:** **Action:** Turns the Siren **ON**.
- **Panic State Cleared:** **Action:** Turns the Siren **OFF**.

### Contact Sensors (Door/Window)
- **Sensor Opened:** Evaluated as a breach. **Action:** Turns the Siren **ON**.
- **Sensor Closed:** **Action:** Turns the Siren **OFF**.

### Aqara FP300 Presence Sensor (Occupancy)
- **Presence Detected in a Zone:** Logs `"🚶 [USECASE] Person detected in zone..."`. 
  - *Note: If ALL registered physical sensors are occupied, it logs `"OCCUPANCY in ALL physical sensors"`.*
- **Presence Cleared in a Zone:** Logs `"💨 [USECASE] Occupancy cleared in zone..."`.
- *(Note: Siren triggering for presence events is currently **disabled** in code).*

### Generic Light Sensors
- **Light Turned ON:** Logs `"☀️ [USECASE] Light turned ON"`. *(Siren trigger disabled)*.
- **Light Turned OFF:** Logs `"🌙 [USECASE] Light turned OFF"`. *(Siren trigger disabled)*.

### Generic Buttons (On/Off/Toggle)
- **Button Sends ON:** **Action:** Turns the Siren **ON**.
- **Button Sends OFF:** **Action:** Turns the Siren **OFF**.
- **Button Sends TOGGLE:** **Action:** Toggles the current Siren state (if OFF, turns ON; if ON, turns OFF).

---

## 🚨 3. Siren Logic Summary

For quick reference, here are the explicit conditions that affect the Siren state:

**When does the Siren turn ON?**
1. 3-presses within 3 seconds on an Aqara Button.
2. An Onics Panic Button triggers a panic state.
3. A Contact Sensor opens.
4. A generic button sends an `ON` or `TOGGLE` (when currently off) command.
5. User runs the `siren on` CLI command.

**When does the Siren turn OFF?**
1. 1-press on an Aqara Button *while* the siren is currently active.
2. An Onics Panic Button clears its panic state.
3. A Contact Sensor closes.
4. A generic button sends an `OFF` or `TOGGLE` (when currently on) command.
5. User runs the `siren off` CLI command.
