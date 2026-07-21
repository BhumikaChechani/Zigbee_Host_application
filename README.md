# ZNP MT Host Controller (C)

Welcome to the **ZNP MT Host Controller**, a lightweight Linux application designed to drive a CC1352P7 Zigbee Coordinator. 

This application acts as the "brain" of your Zigbee network. It communicates with the CC1352P7 over a serial connection (MT/UART protocol), treating the chip purely as a radio bridge. All the smart logic—like device pairing, message routing, and automation—runs directly on your Linux host!

### Supported Devices
- **Smart Siren (SIRZB-110)** (IAS WD cluster `0x0502`)
- **Onics Smart/Panic Button (SBTZB-110)** (IAS Zone `0x0500` + On/Off `0x0006`)
- **Aqara Wireless Mini Switch T1** (On/Off `0x0006`)
- **Aqara FP300 Presence Sensor** (Occupancy `0x0406`, Light Sensing, + custom clusters)
- **Contact Sensors**

---

## 🚀 Quick Start Guide

### 1. Install Dependencies
Before compiling, you need to install the C compiler, make utility, and serial diagnostic tools. Run the automated setup script included in this repository (supports Debian/Ubuntu, Fedora, CentOS, and Red Hat systems):

```sh
sudo ./install_dependencies.sh
```

*Note: After the script completes, please log out and log back in (or reboot) so the serial port group permissions take effect.*

### 2. Build the Application
The project is written in standard C11 and uses a simple Makefile. To compile the code into an executable:

```sh
# Compile the project
make

# (Optional) To clean up old build files before recompiling:
make clean
```
*This will generate the final executable at `bin/znp_host_c`.*

### 3. Run the Controller
You **must** run the application from the root `host_c/` folder. This ensures the `devices.txt` registry file is saved and loaded correctly from the current directory.

```sh
# Standard run (uses default port /dev/ttyACM0 and boots from NVRAM)
./bin/znp_host_c

# Run on a specific serial port
./bin/znp_host_c /dev/ttyUSB0

# Factory reset (Wipes NVRAM, rewrites Zigbee config, and forms a brand new network)
./bin/znp_host_c -f
```

---

## 💻 CLI Commands

Once the application is running, you can interact with the Zigbee network directly by typing commands into your terminal. 

### General & System Management
| Command | Description |
| :--- | :--- |
| **`help`** | Prints a list of all available commands. |
| **`status`** | Displays a summary of all registered devices, their network addresses, last seen times, and active configurations. |
| **`discover [addr hex]`** | Broadcasts discovery requests to find new devices on the network. If a specific address is provided, it actively queries only that device. |
| **`permit [duration]`** | Opens the Zigbee network for new devices to join. Defaults to 60 seconds. *(Example: `permit 120` opens it for 2 minutes).* |
| **`exit`** or **`quit`** | Safely closes the serial connection and exits the application. |

### Sensor Configuration
| Command | Description |
| :--- | :--- |
| **`env <addr>`** | Fetches current environmental readings (e.g., Temp, Humidity, Battery) for supported sensors (Aqara Occupancy, Frient Vibration). Note: Battery-powered sensors must be awake. |
| **`sensitivity <addr> <level>`** | Sets the sensor's physical sensitivity. For Aqara Occupancy: 1=Low, 2=Med, 3=High. For Frient Vibration: 1=Most Sensitive to 15=Least Sensitive (Default 10). |
| **`forcesetup <addr>`** | Forces a full re-initialization and Zigbee binding setup for the sensor. |

### Aqara FP300 Presence Sensor
| Command | Description |
| :--- | :--- |
| **`zone <addr> <idx> <start> <end>`** | Configures a detection zone. Distance slices are 25cm each.<br>*(Example: `zone 7AF2 0 0 2` configures zone 0 to cover 0-50cm).* |
| **`zonedel <addr> <idx>`** | Deletes a previously configured zone. |
| **`spatiallearn <addr>`** | Triggers the sensor's spatial learning calibration (ensure the room is completely empty before running). |
| **`lightthreshold [value]`** | Sets or displays the light intensity threshold (lux index) for Aqara Occupancy light sensing (Default: 10000). |

### Siren Controls
| Command | Description |
| :--- | :--- |
| **`siren on`** | Turns **all** sirens ON simultaneously. |
| **`siren off`** | Turns **all** sirens OFF simultaneously. |
| **`siren vol <0-3>`** | Sets the global siren volume (0 = Low, 1 = Medium, 2 = High, 3 = Very High). |
| **`siren mode <1-6>`** | Sets the global siren sound mode (1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic). |
| **`siren test <addr> [mode]`** | Sends a test warning to a specific siren. Can optionally override the global mode for testing. |

---

## 🏗️ Architecture Overview

The design separates three core responsibilities to ensure stability and responsiveness:
1. **Transport**: Manages the serial link to the Zigbee chip.
2. **Sensors**: Handles device-specific protocols (e.g., Siren, Button, Presence).
3. **Use-Case**: Enforces system policies and automations.

Each sensor runs on its own background thread, and the central policy engine runs on its own thread. This ensures that a slow or blocked device operation will never stall the rest of the system!

```text
                         ┌──────────────────────────────────────────┐
   serial /dev/ttyACM0   │            TRANSPORT  (znp_host.c)         │
        │                │                                            │
        │  bytes         │  reader_thread ── parse MT frame + FCS ──┐ │
        └───────────────►│                                          │ │
                         │   SRSP ─► satisfies the blocking sreq()  │ │
                         │   AREQ ─► event_queue (ring buffer)      │ │
                         └───────────────────────────┬──────────────┘ │
             any thread ── sreq() (mutex-serialized) ─┘                │
                                                     event_queue       │
                                                          │            │
                         ┌────────────────────────────────▼────────────┐
                         │        DISPATCHER   (main.c main loop)       │
                         │  • ZDO discovery → classify device type      │
                         │  • maintain registries (synchronous)         │
                         │  • route AF messages to the owning sensor    │
                         │  • permit-join auto-refresh                  │
                         └───────┬───────────────┬───────────────┬──────┘
              post_assign/post_af│               │               │
                 ┌───────────────▼──┐   ┌─────────▼────────┐   ┌──▼───────────────┐
                 │  SIREN thread    │   │  AQARA thread    │   │  ONICS thread    │
                 │  (siren.c)       │   │ (aqara_button.c) │   │(onics_button.c)  │
                 │  inbox + worker  │   │  inbox + worker  │   │  inbox + worker  │
                 │  assign→setup    │   │  assign→setup    │   │  assign→setup    │
                 │  (write CIE)     │   │  (bind On/Off)   │   │  (bind+CIE+activ)│
                 │  af→tamper enroll│   │  af→parse press  │   │  af→press/enroll/│
                 │                  │   │        │         │   │       panic  │   │
                 └───────▲──────────┘   └────────┼─────────┘   └───────────┼──────┘
                         │ siren_control_all()   │ usecase_post()          │
                         │                        ▼                        ▼
                         │              ┌──────────────────────────────────────┐
                         └──────────────┤   USE-CASE thread   (usecase.c)       │
                                        │   collects UC_BUTTON_* / UC_PANIC_*   │
                                        │   events → drives sirens              │
                                        └──────────────────────────────────────┘
```

### How a Button Press Works (Data Flow)
1. **Receive**: The `reader` thread receives bytes from the serial port, frames them into a Zigbee ZCL message, and queues it.
2. **Dispatch**: The `dispatcher` thread reads the queue and routes the message to the specific sensor's inbox (e.g., `onics_button.c`) without blocking.
3. **Parse**: The sensor thread parses the raw Zigbee On/Off command and translates it into a high-level event (e.g., `UC_BUTTON_ON`).
4. **Act**: The `usecase` thread receives the `UC_BUTTON_ON` event and executes the policy (e.g., calling `Siren_ControlAll(1)` to sound the alarm).

*(Notice how the button never talks directly to the siren—everything goes through the `usecase.c` logic layer!)*

### Security & Health Monitoring
The system includes built-in safeguards to ensure network reliability:
- **Active Health Watchdog**: A background thread actively tracks the `last_seen` timestamp of all registered devices. 
  - Sleepy battery-powered devices (buttons, contact sensors) are marked **OFFLINE** if they miss their check-ins for >2 hours.
  - Active routers (sirens, occupancy sensors) are polled every 60 seconds and marked **OFFLINE** if they stop responding for >5 minutes.
  - The `[HEALTH]` logs will instantly warn the user of unreachable devices, and the system prevents sending commands to unreachable sirens.
- **Hardware Tamper Detection**: Devices equipped with physical tamper switches (Frient Contact Sensors, Frient Vibration Sensors, Smart Sirens, and Onics Panic Buttons) are actively monitored. If the battery cover is opened or the device is ripped off the wall, the system instantly triggers a `[SECURITY] TAMPER DETECTED` alarm and fires all sirens at full volume.

### Adding a New Sensor
The architecture is designed to scale easily:
1. Create a `sensor_foo.c` and `sensor_foo.h` (copy an existing one like `aqara_button.c` as a template).
2. Set up its registry array, worker thread, and Zigbee cluster bindings.
3. Update `main.c` to recognize its Simple-Descriptor cluster.
4. Have the new sensor emit high-level events to `usecase.c` for system actions.

No Makefiles need to be modified—`make` automatically compiles all `.c` files in the `src/` directory!
