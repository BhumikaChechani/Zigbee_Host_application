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

### Sensor Configuration
| Command | Description |
| :--- | :--- |
| **`env <addr>`** | Fetches current environmental readings (e.g., Temp, Humidity, Battery) for supported sensors (Aqara Occupancy, Frient Vibration). Note: Battery-powered sensors must be awake. |
| **`sensitivity <addr> <level>`** | Sets the sensor's physical sensitivity. For Aqara Occupancy: 1=Low, 2=Med, 3=High. For Frient Vibration: 1=Most Sensitive to 15=Least Sensitive (Default 10). |
| **`forcesetup <addr>`** | Forces a full re-initialization and Zigbee binding setup for the sensor. |

### Aqara FP300 Presence Sensor
| Command | Description |
| :--- | :--- |
| **`zone <addr> <idx> <start> <end>`** | Configures a detection zone. Distance slices are 25cm each (e.g., `zone 7AF2 0 0 2` covers 0-50cm). |
| **`zonedel <addr> <idx>`** | Deletes a previously configured zone. |
| **`spatiallearn <addr>`** | Triggers the sensor's spatial learning calibration (ensure room is empty first). |

### Siren Controls
| Command | Description |
| :--- | :--- |
| **`siren on`** | Turns **all** sirens ON simultaneously. |
| **`siren off`** | Turns **all** sirens OFF simultaneously. |
| **`siren vol <0-3>`** | Sets the global siren volume (0 = Low, 1 = Medium, 2 = High, 3 = Very High). |
| **`siren mode <1-6>`** | Sets the global siren sound mode (1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic). |
| **`siren test <addr> [mode]`** | Sends a test warning to a specific siren. Can optionally override the global mode for testing. |

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

### Frient/Develco Siren (SIRZB-110) Tamper
- **Tamper Switch Opened:** Evaluated as hardware sabotage. **Action:** Turns the Siren **ON** (full alarm).
- **Tamper Switch Closed:** **Action:** Turns the Siren **OFF**.

### Frient Vibration Sensor (WISZB-13x)
- **Movement/Tilt (Alarm 1) Detected:** **Action:** Turns the Siren **ON** (full alarm).
- **Movement/Tilt (Alarm 1) Cleared:** **Action:** Turns the Siren **OFF** (auto-cleared after 5s of no movement).
- **Vibration (Alarm 2) Detected:** **Action:** Turns the Siren Squawk **ON** (short chime).
- **Vibration (Alarm 2) Cleared:** **Action:** (No action needed, Squawk is self-terminating).

### Contact Sensors (Door/Window)
- **Sensor Opened:** Evaluated as a breach. **Action:** Turns the Siren Strobe **ON** (Silent Visual Alarm).
- **Sensor Closed:** **Action:** Turns the Siren Strobe **OFF**.

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
3. Frient Vibration Sensor detects Movement/Tilt (Alarm 1).
4. Siren Tamper switch opens (sabotage).
5. A generic button sends an `ON` or `TOGGLE` (when currently off) command.
6. User runs the `siren on` CLI command.

**When does the Siren Strobe (Silent Alarm) turn ON?**
1. A Contact Sensor opens.

**When does the Siren turn OFF?**
1. 1-press on an Aqara Button *while* the siren is currently active.
2. An Onics Panic Button clears its panic state.
3. Frient Vibration Sensor clears its Movement/Tilt state (Alarm 1).
4. Siren Tamper switch closes.
5. A generic button sends an `OFF` or `TOGGLE` (when currently on) command.
6. User runs the `siren off` CLI command.

**When does the Siren Strobe (Silent Alarm) turn OFF?**
1. A Contact Sensor closes.
