# 🧪 QA Testing Matrix (Sensor-Wise)

This sheet is designed for straightforward QA testing. Follow the **Test Action** and verify that the **Expected Result** occurs.

---

### 🟢 1. Aqara Wireless Mini Switch (Smart Button)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Press button 1 time | Console logs single press. **No siren.** | Debounce is set to 0.10s to allow fast tapping. |
| Press button 2 times | Console logs double press. **No siren.** | |
| Press button 3 times **FAST** (within 3 secs) | 🚨 **FULL SIREN ALARM** on all sirens. | Simulates SOS. Sirens are forced to MAX volume. |
| Press button 3 times **SLOW** (> 3 secs gap) | Console resets history. **No alarm.** | Tests timeout rejection logic. |

---

### 🔴 2. Onics Panic Button
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Press the RED panic button | 🚨 **FULL SIREN ALARM** on all sirens. | Tests IAS Zone panic activation mapping. |
| Press the RED panic button again | 🔇 **ALL SIRENS STOP**. | Tests IAS Zone panic clear mapping. |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage**. | Tests generic sensor health data. |

---

### 🚪 3. Contact Sensor (Door/Window)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Move magnet away (Open Door) | 🔔 **DOUBLE BEEP** (2 fast chirps) on sirens. | Beeps use Burglar mode with 20ms ON time and 300ms gap. |
| Bring magnet close (Close Door) | 🔔 **SINGLE BEEP** (1 fast chirp) on sirens. | |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage** & **Temperature**. | Tests generic sensor health data. |

---

### 📳 4. Frient Vibration Sensor (Glass Break)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Tap glass gently (Vibration / Alarm 2) | 🚨 **FULL SIREN ALARM** (Emergency Panic) on all sirens. | Triggers Mode 6 sound. |
| Shake or tilt heavily (Movement / Alarm 1) | 🚨 **FULL SIREN ALARM** (Emergency Panic) on all sirens. | Critical breach / window broken. Triggers Mode 6 sound. |
| Remove battery cover (Tamper switch) | 🚨 **FULL SIREN ALARM** on all sirens. | Protects device from being dismantled. |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage** & **Temperature**. | **WAKEUP REQUIRED:** You must tap/vibrate the sensor right before hitting enter to wake its radio! |
| Type `sensitivity <addr> <1-15>` | ⚙️ Configures hardware sensitivity. | 1=Most sensitive, 15=Least sensitive (Default 10). |

---

### 🚨 5. Frient Smart Siren (SIRZB-110)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Remove mounting backplate (Tamper switch) | 🚨 **FULL SIREN ALARM** on all sirens. | Siren hardware tamper detection. |
| Type `env <addr>` in CLI | 📊 Prints **Battery/Voltage**. | Fetches diagnostics from the siren. |
| Type `siren on` and hit Enter | 🚨 **FULL SIREN ALARM** on all sirens. | Tests global siren activation. |
| Type `siren off` and hit Enter | 🔇 **ALL SIRENS STOP** immediately. | Stops the global test. |
| Type `siren mode <1-6>` | 🎶 Sets global siren sound mode. | **Modes:** 1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic. |
| Type `siren vol <0-3>` | 🔊 Sets global siren volume level. | **Levels:** 0=Low, 1=Medium, 2=High, 3=Very High. Note: Some firmwares ignore volume. |
| Type `siren test <addr> [mode]` | 🚨 Sounds ONLY the specific siren address. | Provide mode 1-6 to test a specific sound (1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic). |
| Type `siren stop <addr>` | 🔇 **ONLY THAT SPECIFIC SIREN** stops. | Stops a siren test. |

---

### 🚶 6. Aqara FP300 Presence Sensor
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Walk into the room / zone | Console logs person detected. | Siren logic is disabled for testing. |
| Type `env <addr>` in CLI | 📊 Prints **Temperature** & **Humidity**. | Tests FP300 environmental data. |
| Type `zone <addr> <idx> <start> <end>` | 📏 Configures a specific detection zone. | Start/End are in 25cm slices. |
| Type `zonedel <addr> <idx>` | 🗑️ Deletes the specific detection zone. | |
| Type `spatiallearn <addr>` | 📡 Triggers spatial background learning. | Ensure the room is empty first! |
| Type `sensitivity <addr> <1\|2\|3>` | ⚙️ Configures radar sensitivity. | 1=Low, 2=Medium, 3=High. |

---

### 💻 7. Global CLI Commands (Controller Level)
| Test Action (What to do) | Expected Result (System Response) | Notes |
| :--- | :--- | :--- |
| Type `status` and hit Enter | 📋 Prints list of all connected devices. | Shows online status and last-seen time. |
| Type `permit [seconds]` | 🔓 Opens the Zigbee network for pairing. | Default is 60s if not specified. |
| Type `discover <addr>` | 🔍 Discovers device endpoints and clusters. | Forces Zigbee active endpoint discovery. |
| Type `forcesetup <addr>` | ⚙️ Re-runs initial configuration binding. | Fixes devices that didn't set up correctly. |
