#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Feature Toggles: Enable or disable specific sensor modules at compile-time.
// Set to 1 to compile and run the module, set to 0 to completely remove it.
// ============================================================================
#define APP_VERSION "2.3.1"

// ============================================================================

#define ENABLE_SIREN 1
#define ENABLE_AQARA_BUTTON 1
#define ENABLE_ONICS_BUTTON 1
#define ENABLE_CONTACT_SENSOR 1
#define ENABLE_AQARA_OCCUPANCY 1
#define ENABLE_VIBRATION_SENSOR 1
#define ENABLE_OKOS_SIREN 1  ///< Okos Smart Siren (Tuya/Zigbee 3.0) — set 1 to enable
#define ENABLE_AQARA_TVOC 1  ///< Aqara Air Quality Sensor (TVOC AAQS-S01) — set 1 to enable

#endif // CONFIG_H
