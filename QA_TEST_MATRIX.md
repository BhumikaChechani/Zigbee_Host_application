#  QA Testing Matrix (Sensor-Wise)

This sheet is designed for straightforward QA testing. Follow the **Test Action** and verify that the **Expected Result** occurs.

---

###  1. Aqara Wireless Mini Switch (Smart Button)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Press button 1 time | Console logs single press. **If sirens are ON, they turn OFF.** | Debounce is set to 0.10s to allow fast tapping. |
| Press button 2 times | Console logs double press. **No siren.** | |
| Press button 3 times **FAST** (within 3 secs) |  **FULL SIREN ALARM** (Burglar / Mode 1) on all sirens. | Simulates SOS. Sirens are forced to MAX volume and Burglar mode. |
| Press button 3 times **SLOW** (> 3 secs gap) | Console resets history. **No alarm.** | Tests timeout rejection logic. |

---

###  2. Onics Panic Button
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Press the RED panic button | 🚨 **FULL SIREN ALARM** (Global Alarm Mode) on all sirens. | Tests IAS Zone panic activation mapping. |
| Long press the RED panic button | 🔇 **ALL SIRENS STOP** (Panic Cleared). | Tests IAS Zone panic clear mapping (Long press clears panic). |
| Remove battery cover (Tamper) | 📝 **LOG ONLY** (TAMPER DETECTED). | Tests IAS Zone tamper alarm on cover open. |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage**. | Tests generic sensor health data. |

---

###  3. Contact Sensor (Door/Window)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Move magnet away (Open Door) | 🔔 **DOUBLE BEEP** (2 fast chirps) in Burglar mode. Sets zone door state to **OPEN**. | Beeps use Burglar mode (Mode 1) with 100ms ON time and 300ms gap. |
| Bring magnet close (Close Door) | 🔔 **SINGLE BEEP** (1 fast chirp) in Burglar mode. Sets zone door state to **CLOSED**. | Beep uses Burglar mode (Mode 1) with 100ms ON time. |
| Remove battery cover (Tamper) | 📝 **LOG ONLY** (TAMPER DETECTED). | Tests IAS Zone tamper alarm on cover open. |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage** & **Temperature**. | Tests generic sensor health data. |

---

###  4. Frient Vibration Sensor (Glass Break)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Tap glass gently (Vibration / Alarm 2) |  **FULL SIREN ALARM** (Police Panic / Mode 4) on all sirens. | Triggers Mode 4 sound. Sirens stop when vibration clears. |
| Shake or tilt heavily (Movement / Alarm 1) |  **FULL SIREN ALARM** (Police Panic / Mode 4) on all sirens. | Critical breach / window broken. Triggers Mode 4 sound. Sirens stop when movement clears. |
| Remove battery cover (Tamper switch) | **LOG ONLY** (TAMPER DETECTED). | Protects device from being dismantled. |
| Type `env <addr>` in CLI |  Prints **Battery/Voltage** & **Temperature**. | **WAKEUP REQUIRED:** You must tap/vibrate the sensor right before hitting enter to wake its radio! |
| Type `sensitivity <addr> <1-15>` |  Configures hardware sensitivity. | 1=Most sensitive, 15=Least sensitive (Default 10). |

---

###  5. Frient Smart Siren (SIRZB-110)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Remove mounting backplate (Tamper switch) | 📝 **LOG ONLY** (TAMPER DETECTED). | Tests IAS Zone tamper alarm on wall removal. |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage**. | Light / battery queries. |
| Type `siren on` and hit Enter | 🚨 **FULL SIREN ALARM** on all sirens. | Tests global siren activation. |
| Type `siren off` and hit Enter | 🔇 **ALL SIRENS STOP** immediately. | Stops the global test. |
| Type `siren mode <1-6>` | 🎶 Sets global siren sound mode. | **Modes:** 1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic. |
| Type `siren vol <0-3>` | 🔊 Sets global siren volume level. | **Levels:** 0=Low, 1=Medium, 2=High, 3=Very High. Note: Some firmwares ignore volume. |
| Type `siren test <addr> [mode]` | 🚨 Sounds ONLY the specific siren address. | Provide mode 1-6 to test a specific sound (1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic). |
| Type `siren stop <addr>` | 🔇 **ONLY THAT SPECIFIC SIREN** stops. | Stops a siren test. |

---

###  6. Aqara FP300 Presence Sensor

> [!NOTE]
> **Demo Environment Presets:**
> * **Zone Configuration:** Set zone 0 to slices 5–9 (125cm to 225cm) via command: `zone <addr> 0 5 9`
> * **Light Threshold:** Set threshold to 20000 via command: `lightthreshold <addr> 20000`
> * **Fast Polling:** Polling interval for light and presence is optimized to 300ms for near-instant response.

| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Walk into zone when door is CLOSED | Console logs presence but **no alarm triggers**. | Ignores presence when door is closed in that zone. |
| Walk into zone when door is OPEN |  **SIREN SOUNDS** for 5 seconds on all sirens. | Triggers Mode 5 (Fire Panic) warning. |
| Type `env <addr>` in CLI |  Prints **Temperature** & **Humidity**. | Tests FP300 environmental data. |
| Type `zone <addr> <idx> <start> <end>` | 📏 Configures a specific detection zone. | Start/End are in 25cm slices. |
| Type `zonedel <addr> <idx>` |  Deletes the specific detection zone. | |
| Type `spatiallearn <addr>` |  Triggers spatial background learning. | Ensure the room is empty first! |
| Type `sensitivity <addr> <1\|2\|3>` |  Configures radar sensitivity. | 1=Low, 2=Medium, 3=High. |
| Shine light on sensor (Light ON) |  **FULL SIREN ALARM** (Emergency Panic / Mode 6) on all sirens. | Triggers Mode 6 sound. |
| Cover sensor from light (Light OFF) | **ALL SIRENS STOP**. | |
| Type `lightthreshold <addr> <value>` |  Configures the light sensing trigger threshold. | Set lower for higher sensitivity to light, higher for darker environments. |

---

### 💻 7. Global CLI Commands (Controller Level)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Type `status` and hit Enter | 📋 Prints list of all connected devices. | Shows online status, last-seen, and light readings. |
| Type `permit [seconds]` | 🔓 Opens the Zigbee network for pairing. | Default is 60s if not specified. |
| Type `discover <addr>` | 🔍 Discovers device endpoints and clusters. | Forces Zigbee active endpoint discovery. |
| Type `forcesetup <addr>` | ⚙️ Re-runs initial configuration binding. | Fixes devices that didn't set up correctly. |
| Type `remove <addr>` | 🗑️ Device is removed and kicked. | Tests device removal. |
| Type `rebind <addr>` | 🔗 Re-binds an unresponsive device. | Fixes devices with stale bindings. |
| Disconnect Siren/Occupancy (Wait 5 min) | 🔴 `[HEALTH] Device OFFLINE` | Tests the router active watchdog timeout. |
| Remove Sensor Battery (Wait 2 hours) | 🔴 `[HEALTH] Device OFFLINE` | Tests the sleepy sensor watchdog timeout. |

---

###  8. System Alarm & Siren Sound Matrix

The table below lists all conditions that trigger or modify the sirens, along with the specific siren mode and sound profile activated.

| Event / Trigger Condition | Target Action | Siren Warning Mode | Sound Profile Description |
| :--- | :--- | :--- | :--- |
| **Aqara Button:** 3 Fast Presses | Sirens ON (Full Volume) | **Mode 1 (Burglar)** | Burglar alarm sound |
| **Aqara Button:** 1 Press | Sirens OFF | **Mode 0 (Stop)** | Silence |
| **Onics Button:** Red Button Press | Sirens ON | **Global Active Mode** (Default: 1) | Active global sound (Burglar by default) |
| **Onics Button:** Long Press (Clear) | Sirens OFF | **Mode 0 (Stop)** | Silence |
| **Contact Sensor:** Magnet open (Door Open) | Double Beep (Chime) & sets door state to OPEN | **Mode 1 (Burglar)** | 2 short beeps (100ms ON / 300ms gap) |
| **Contact Sensor:** Magnet close (Door Close) | Single Beep (Chime) & sets door state to CLOSED | **Mode 1 (Burglar)** | 1 short beep (100ms ON) |
| **Presence Sensor:** Person detected + Door OPEN | Sirens ON for 5 seconds | **Mode 5 (Fire Panic)** | Fire Panic sound |
| **Presence Sensor:** Person detected + Door CLOSED | Sirens ignored (Ignored for Zone) | - | Silence |
| **Vibration Sensor:** Vibration detected | Sirens ON | **Mode 4 (Police Panic)** | Police Panic sound |
| **Vibration Sensor:** Vibration cleared | Sirens OFF | **Mode 0 (Stop)** | Silence |
| **Vibration Sensor:** Movement detected | Sirens ON | **Mode 4 (Police Panic)** | Police Panic sound |
| **Vibration Sensor:** Movement cleared | Sirens OFF | **Mode 0 (Stop)** | Silence |
| **Light Sensor:** Light ON | Sirens ON | **Mode 6 (Emergency Panic)** | Emergency Panic sound |
| **Light Sensor:** Light OFF | Sirens OFF | **Mode 0 (Stop)** | Silence |
| **Tamper:** Cover opened / Wall removed | Logs `[SECURITY] TAMPER DETECTED` | - | Console log only |
| **Tamper:** Cover closed | Logs `Tamper CLEARED` | - | Console log only |
