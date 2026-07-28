# Okos Smart Siren — Feature List & Integration Status

## Device Overview
| Property        | Value                                |
|----------------|--------------------------------------|
| Manufacturer    | Okos Technologies (Tuya ecosystem)  |
| Connectivity    | Zigbee 3.0                          |
| Profile         | Home Automation (0x0104)            |
| Default EP      | 0x01                                |
| Primary Cluster | IAS Warning Device (0x0502)         |
| Power           | USB 5V/1A + 2×CR123A battery backup |
| Max Volume      | 100 dB                              |
| Alarm Tones     | 18 selectable tones                 |

## ZCL Clusters
| Cluster | ID     | Role   | Description                        |
|---------|--------|--------|------------------------------------|
| Basic              | 0x0000 | Server | Manufacturer / model info   |
| Power Config       | 0x0001 | Server | Battery backup percentage   |
| Identify           | 0x0003 | Server | Locate device (flash/beep)  |
| IAS Zone           | 0x0500 | Server | Tamper zone enroll & status |
| IAS Warning Device | 0x0502 | Server | Start Warning / Squawk      |
| Temperature        | 0x0402 | Server | Built-in temp sensor        |
| Humidity           | 0x0405 | Server | Built-in humidity sensor    |
| OTA Upgrade        | 0x0019 | Client | Firmware OTA updates        |
| Tuya Cluster       | 0xEF00 | Server | Tone selection (mfr-specific)|

## Feature Integration Phases

### Phase 1 — Core Alarm (CURRENT)
- [x] Device discovery (Simple Descriptor: IAS WD cluster 0x0502)
- [x] IAS CIE address write (pairing / zone enroll setup)
- [x] Zone Enroll Request handling (cluster 0x0500, cmd 0x01)
- [x] Zone Status Change — tamper detection (bit 2)
- [x] Start Warning (alarm ON) via `OkosSiren_ControlAll()`
- [x] Stop Warning (alarm OFF) via `OkosSiren_ControlAll(0)`
- [x] Per-device alarm control via `OkosSiren_Control(addr, mode)`
- [x] Volume control (0=Low ~70dB, 1=Med ~85dB, 2=High ~100dB)
- [x] Beep sequences (arm/disarm acknowledgement chirps)
- [x] Persist settings in devices.txt (volume, toneId, strobe)
- [x] Offline detection (5-min watchdog, keep-alive poll every 60s)

### Phase 2 — Tone & Strobe (TODO)
- [ ] 18-tone selection via Tuya cluster 0xEF00 (dp=2, type=enum)
- [ ] CLI command: `okos tone <addr> <1-18>`
- [ ] Strobe-only mode (warnMode=0, strobeMode=STROBE_USE_LEVEL)
- [ ] Strobe level control (0=Low, 1=Med, 2=High, 3=Very High)
- [ ] CLI command: `okos strobe <addr> <0|1>`

### Phase 3 — Environmental Sensor (TODO)
- [ ] Temperature read-on-demand (cluster 0x0402, attr 0x0000)
- [ ] Humidity read-on-demand (cluster 0x0405, attr 0x0000)
- [ ] CLI command: `okos env <addr>`
- [ ] Attribute reporting configuration (bind + configure reporting)
- [ ] Auto-log temperature/humidity on report-change frames

### Phase 4 — Battery Monitoring (TODO)
- [ ] Battery percentage read (cluster 0x0001, attr 0x0021)
- [ ] CLI command: `okos battery <addr>`
- [ ] Low-battery event → UC_BATTERY_LOW use-case event
- [ ] Display battery % in `status` CLI output

### Phase 5 — OTA Firmware Upgrade (TODO)
- [ ] OTA cluster (0x0019) image query response
- [ ] CLI command: `okos ota <addr> <firmware.ota>`

## Alarm Tones Reference (1-18)
| ID | Name              | Use Case                     |
|----|-------------------|------------------------------|
|  1 | Burglar           | Intrusion alarm (default)    |
|  2 | Fire              | Fire alarm                   |
|  3 | Emergency         | Generic emergency            |
|  4 | Police Panic      | Panic button trigger         |
|  5 | Fire Panic        | Fire panic                   |
|  6 | Emergency Panic   | Emergency panic              |
|  7 | Doorbell 1        | Entry notification           |
|  8 | Doorbell 2        | Secondary entry notification |
|  9 | Beep Fast         | Arm acknowledgement          |
| 10 | Beep Slow         | Disarm acknowledgement       |
| 11 | Siren High        | High-pitch continuous        |
| 12 | Siren Low         | Low-pitch continuous         |
| 13 | Sweep             | Frequency sweep              |
| 14 | Pulse             | Pulsed alert                 |
| 15 | Warble            | Warble tone                  |
| 16 | Chirp             | Short squawk emulation       |
| 17 | Cuckoo            | Novelty / custom             |
| 18 | Custom            | Reserved custom slot         |

## Persistence Format (devices.txt)
```
okos_siren <ShortAddr> <EP> <IEEE16hex> <hasIeee> <zoneId> <volume> <toneId> <strobeMode>
```
Example:
```
okos_siren 1234 01 AABBCCDDEEFF0011 1 3 2 1 0
```

## Files
| File                          | Purpose                             |
|-------------------------------|-------------------------------------|
| `include/okos_siren.h`        | Public API + structs + constants    |
| `src/okos_siren.c`            | Full implementation                 |
| `OKOS_SIREN_FEATURES.md`      | This document                       |

## config.h Toggle
```c
#define ENABLE_OKOS_SIREN 1   // Set to 0 to exclude from build
```

## Next Integration Steps
1. Add `ENABLE_OKOS_SIREN` to `include/config.h`
2. Add `okos_siren` load/save blocks to `Device_Save()` / `Device_Load()` in `main.c`
3. Add `OkosSiren_Init()` / `OkosSiren_Start()` to `main()` in `main.c`
4. Add `OkosSiren_IsKnown()` / routing in `Main_HandleIncomingFrame()` in `main.c`
5. Add `OkosSiren_UpdateIeee()` call in the IEEE response handler in `main.c`
6. Add CLI commands (`okos on`, `okos off`, `okos tone`, `okos env`, etc.) in `cli.c`
7. Wire `OkosSiren_ControlAll()` into the use-case trigger events in `usecase.c`
