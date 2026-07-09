#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// Feature Toggles: Enable or disable specific sensor modules at compile-time.
// Set to 1 to compile and run the module, set to 0 to completely remove it.
// ============================================================================

#define ENABLE_SIREN 1
#define ENABLE_AQARA_BUTTON 1
#define ENABLE_ONICS_BUTTON 1
#define ENABLE_CONTACT_SENSOR 1
#define ENABLE_AQARA_OCCUPANCY 1
#define ENABLE_VIBRATION_SENSOR 1

#endif // CONFIG_H
