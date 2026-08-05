#include "cli.h"
#include "znp_host.h"
#include "config.h"

#include "aqara_button.h"
#include "aqara_occupancy.h"
#include "contact_sensor.h"
#include "vibration_sensor.h"
#include "logger.h"

#if ENABLE_SIREN
#include "siren.h"
#endif
#if ENABLE_AQARA_BUTTON
#include "aqara_button.h"
#endif
#if ENABLE_ONICS_BUTTON
#include "onics_button.h"
#endif
#if ENABLE_AQARA_OCCUPANCY
#include "aqara_occupancy.h"
#endif
#if ENABLE_CONTACT_SENSOR
#include "contact_sensor.h"
#endif
#if ENABLE_VIBRATION_SENSOR
#include "vibration_sensor.h"
#endif
#if ENABLE_OKOS_SIREN
#include "okos_siren.h"
#endif
#if ENABLE_AQARA_TVOC
#include "aqara_tvoc.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

static void Cli_HandleCommand( const char *cmd_ )
{
    char cmdCopy[256];
    strncpy( cmdCopy, cmd_, sizeof( cmdCopy ) );
    cmdCopy[sizeof( cmdCopy ) - 1] = '\0';
    char *parts[16];
    int numParts = 0;
    char *token = strtok( cmdCopy, " \t" );
    while ( token != NULL && numParts < 16 ) { parts[numParts++] = token; token = strtok( NULL, " \t" ); }
    if ( numParts == 0 ) return;
    for ( int i = 0; parts[0][i]; i++ ) { if ( parts[0][i] >= 'A' && parts[0][i] <= 'Z' ) parts[0][i] = parts[0][i] + 32; }

    const char *base = parts[0];
    if ( strcmp( base, "help" ) == 0 )
    {
        printf( "\n--- Available Commands ---\n" );
        printf( "  status                         - Print system and device status\n" );
        printf( "  permit [seconds]               - Open network for joining (default 60s)\n" );
        printf( "  discover [addr]                - Trigger endpoint/cluster discovery\n" );
        printf( "  siren test <addr> [mode]       - Briefly test a siren (mode 1-6)\n" );
        printf( "  siren stop <addr>              - Stop a specific siren\n" );
        printf( "  siren on <addr>                - Turn a specific siren ON\n" );
        printf( "  siren off <addr>               - Turn a specific siren OFF\n" );
        printf( "  siren vol <addr> <0-3>         - Set a specific siren's volume\n" );
        printf( "  siren mode <addr> <1-6>        - Set a specific siren's warning mode\n" );
        printf( "                                   * 1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic\n" );
        printf( "\n--- Okos Smart Siren ---\n" );
        printf( "  okos on  [addr]                - Turn Okos siren(s) ON (omit addr = all)\n" );
        printf( "  okos off [addr]                - Turn Okos siren(s) OFF (omit addr = all)\n" );
        printf( "  okos vol <addr> <0-2>          - Set volume (0=Low, 1=Med, 2=High/100dB)\n" );
        printf( "  okos tone <addr> <1-18>        - Set alarm tone (1=Burglar...18=Custom)\n" );
        printf( "\n--- Sensor Configuration ---\n" );
        printf( "  env <addr>                     - Fetch environment data (Temp/Humidity/Battery)\n" );
        printf( "                                   * Works for: Aqara Occupancy, Frient Vibration, Frient Siren, Okos Siren, Aqara TVOC\n" );
        printf( "  sensitivity <addr> <level>     - Set physical sensitivity level\n" );
        printf( "                                   * Aqara Occupancy: 1=Low, 2=Medium, 3=High\n" );
        printf( "                                   * Frient Vibration: 1=Most sensitive ... 15=Least sensitive (Default 10)\n" );
        printf( "  zone <addr> <id> <min> <max>   - Set detection zone distances (cm)\n" );
        printf( "                                   * Works for: Aqara Occupancy\n" );
        printf( "  zonedel <addr> <id>            - Delete a detection zone (Aqara Occupancy)\n" );
        printf( "  spatiallearn <addr>            - Trigger AI Spatial Learning (Aqara Occupancy)\n" );
        printf( "  forcesetup <addr>              - Force re-bind and config payload to sensor\n" );
        printf( "  onicsdelay <addr> <ms>         - Write ButtonPressActionDelay (attr 0x8001)\n" );
        printf( "  lightthreshold <addr> <val>    - Set light threshold for an Aqara Occupancy sensor\n" );
        printf( "  remove <addr>                  - Send a ZDO Leave Request to forcefully remove a device\n" );
        printf( "  rebind <addr>                  - Force re-send Zigbee bindings/config without removing\n" );
        printf( "  exit                           - Quit application\n\n" );
    }
    else if ( strcmp( base, "status" ) == 0 )
    {
        pthread_mutex_lock( &g_deviceMutex );
        printf( "\n\033[1;35m============================================================\033[0m\n" );
        printf( "\033[1;32m                       SYSTEM STATUS                        \033[0m\n" );
        printf( "\033[1;35m============================================================\033[0m\n" );
        pthread_mutex_unlock( &g_deviceMutex );
#if ENABLE_SIREN
        Siren_PrintStatus();
#endif
#if ENABLE_AQARA_BUTTON
        AqaraButton_PrintStatus();
#endif
#if ENABLE_ONICS_BUTTON
        OnicsButton_PrintStatus();
#endif
#if ENABLE_CONTACT_SENSOR
        ContactSensor_PrintStatus();
        ContactSensor_RefreshIfStale(0.0);
#endif
#if ENABLE_VIBRATION_SENSOR
        VibrationSensor_PrintStatus();
#endif
#if ENABLE_AQARA_OCCUPANCY
        AqaraOccupancy_PrintStatus();
#endif
#if ENABLE_OKOS_SIREN
        OkosSiren_PrintStatus();
#endif
#if ENABLE_AQARA_TVOC
        AqaraTvoc_PrintStatus();
#endif
        printf( "\n\033[1;35m============================================================\033[0m\n\n" );
    }
    else if ( strcmp( base, "siren" ) == 0 )
    {
        if ( numParts < 3 )
        {
            printf( "Usage: siren [test|stop|on|off|vol|mode] <addr> [args...]\n" );
            return;
        }
#if ENABLE_SIREN
        char *sub = parts[1];
        if ( strcmp( sub, "test" ) == 0 )
        {
            uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
            uint8_t ep = Siren_GetEndpoint(addr);
            if ( ep == 0 )
            {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'siren test' command.\n", addr, devName);
                }
            }
            else
            {
                uint8_t testMode = (numParts >= 4) ? (uint8_t)strtol( parts[3], NULL, 10 ) : Siren_GetMode(addr);
                ZNP_SendSirenWarning( addr, ep, 0xBB, testMode, Siren_GetVolume(addr), 240 );
                printf( "Sent warning test to 0x%04X (ep 0x%02X, mode %d)\n", addr, ep, testMode );
            }
        }
        else if ( strcmp( sub, "stop" ) == 0 )
        {
            uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
            uint8_t ep = Siren_GetEndpoint(addr);
            if ( ep == 0 )
            {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'siren stop' command.\n", addr, devName);
                }
            }
            else
            {
                ZNP_SendSirenWarning( addr, ep, 0xBB, 0, Siren_GetVolume(addr), 240 );
                printf( "Sent stop to 0x%04X (ep 0x%02X)\n", addr, ep );
            }
        }
        else if ( strcmp( sub, "on" ) == 0 )
        {
            uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
            if (Siren_GetEndpoint(addr) != 0) {
                Siren_Control( addr, 1 );
                printf( "Sent ON command to Siren 0x%04X.\n", addr );
            } else {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'siren on' command.\n", addr, devName);
                }
            }
        }
        else if ( strcmp( sub, "off" ) == 0 )
        {
            uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
            if (Siren_GetEndpoint(addr) != 0) {
                Siren_Control( addr, 0 );
                printf( "Sent OFF command to Siren 0x%04X.\n", addr );
            } else {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'siren off' command.\n", addr, devName);
                }
            }
        }
        else if ( strcmp( sub, "vol" ) == 0 )
        {
            if ( numParts >= 4 )
            {
                uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
                uint8_t v = (uint8_t)strtol( parts[3], NULL, 10 );
                if (Siren_GetEndpoint(addr) != 0) {
                    Siren_SetVolume( addr, v );
                } else {
                    const char* devName = Device_GetName(addr);
                    if (strcmp(devName, "Unknown Device") == 0) {
                        printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                    } else {
                        printf("Error: The device at 0x%04X (%s) does not support the 'siren vol' command.\n", addr, devName);
                    }
                }
            }
            else
            {
                printf( "Usage: siren vol <addr> <0-3>\n" );
            }
        }
        else if ( strcmp( sub, "mode" ) == 0 )
        {
            if ( numParts >= 4 )
            {
                uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
                uint8_t m = (uint8_t)strtol( parts[3], NULL, 10 );
                if (Siren_GetEndpoint(addr) != 0) {
                    Siren_SetMode( addr, m );
                } else {
                    const char* devName = Device_GetName(addr);
                    if (strcmp(devName, "Unknown Device") == 0) {
                        printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                    } else {
                        printf("Error: The device at 0x%04X (%s) does not support the 'siren mode' command.\n", addr, devName);
                    }
                }
            }
            else
            {
                printf( "Usage: siren mode <addr> <1-6>\n" );
            }
        }
        else
        {
            printf( "ERROR: Unknown siren command '%s'. Type 'help' for usage.\n", sub );
        }
#endif
    }
    else if ( strcmp( base, "okos" ) == 0 )
    {
#if ENABLE_OKOS_SIREN
        if ( numParts < 2 )
        {
            printf( "Usage: okos [on|off|vol|tone] [addr] [args]\n" );
            return;
        }
        const char *sub = parts[1];

        // Pre-emptively validate the address if provided (so we can print detailed errors)
        if ( numParts >= 3 ) {
            uint16_t addr = (uint16_t)strtoul(parts[2], NULL, 16);
            if (!OkosSiren_IsKnown(addr)) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) is not an Okos Siren and does not support 'okos' commands.\n", addr, devName);
                }
                return;
            }
        }

        if ( strcmp( sub, "on" ) == 0 )
        {
            if ( numParts >= 3 )
            {
                uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
                OkosSiren_Control( addr, 1 );
            }
            else { printf( "Usage: okos on <addr>\n" ); }
        }
        else if ( strcmp( sub, "off" ) == 0 )
        {
            if ( numParts >= 3 )
            {
                uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
                OkosSiren_Control( addr, 0 );
            }
            else { printf( "Usage: okos off <addr>\n" ); }
        }
        else if ( strcmp( sub, "vol" ) == 0 )
        {
            if ( numParts >= 4 )
            {
                uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
                uint32_t v = strtoul( parts[3], NULL, 10 );
                if (v > 2) {
                    printf("Error: Invalid volume '%u'. Must be between 0 and 2.\n", v);
                } else {
                    OkosSiren_SetVolume( addr, (uint8_t)v );
                }
            }
            else { printf( "Usage: okos vol <addr> <0-2>\n" ); }
        }
        else if ( strcmp( sub, "tone" ) == 0 )
        {
            if ( numParts >= 4 )
            {
                uint16_t addr = (uint16_t)strtoul( parts[2], NULL, 16 );
                uint32_t t = strtoul( parts[3], NULL, 10 );
                if (t < 1 || t > 18) {
                    printf("Error: Invalid tone '%u'. Must be between 1 and 18.\n", t);
                } else {
                    OkosSiren_SetTone( addr, (uint8_t)t );
                }
            }
            else { printf( "Usage: okos tone <addr> <1-18>\n" ); }
        }

        else
        {
            printf( "ERROR: Unknown okos command '%s'. Type 'help'.\n", sub );
        }
#else
        printf( "Okos siren module is disabled (ENABLE_OKOS_SIREN=0).\n" );
#endif
    }
    else if ( strcmp( base, "zone" ) == 0 )
    {
        if ( numParts >= 5 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            int zoneIdx = (int)strtol( parts[2], NULL, 10 );
            uint32_t startSlice = strtoul( parts[3], NULL, 10 );
            uint32_t endSlice = strtoul( parts[4], NULL, 10 );
            
            uint32_t minCm = startSlice * 25;
            uint32_t maxCm = endSlice * 25;
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetZone( addr, zoneIdx, minCm, maxCm );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'zone' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: zone <addr hex> <zone_idx> <start_slice> <end_slice>\n" );
            printf( "  Each slice is 25cm. Example: zone 7AF2 0 0 1 -> zone 0 is 0-25cm\n" );
            printf( "  Example: zone 7AF2 0 0 2 -> zone 0 is 0-50cm\n" );
        }
    }
    else if ( strcmp( base, "zonedel" ) == 0 )
    {
        if ( numParts >= 3 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            int zoneIdx = (int)strtol( parts[2], NULL, 10 );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_DeleteZone( addr, zoneIdx );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'zonedel' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: zonedel <addr hex> <zone_idx>\n" );
        }
    }
    else if ( strcmp( base, "sensitivity" ) == 0 )
    {
        if ( numParts >= 3 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            uint8_t level = (uint8_t)strtoul( parts[2], NULL, 10 );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetSensitivity( addr, level );
                found = true;
            }
#endif
#if ENABLE_VIBRATION_SENSOR
            if (VibrationSensor_IsKnown(addr)) {
                VibrationSensor_SetSensitivity( addr, level );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'sensitivity' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: sensitivity <addr hex> <1|2|3>  (1=low 2=medium 3=high)\n" );
        }
    }
    else if ( strcmp( base, "spatiallearn" ) == 0 )
    {
        if ( numParts >= 2 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SpatialLearning( addr );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'spatiallearn' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: spatiallearn <addr hex>  (trigger while room is EMPTY)\n" );
        }
    }
    else if ( strcmp( base, "forcesetup" ) == 0 )
    {
        if ( numParts >= 2 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            printf( "Forcing full setup for 0x%04X...\n", addr );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_PostAssign( addr );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'forcesetup' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: forcesetup <addr hex>\n" );
        }
    }
    else if ( strcmp( base, "lightthreshold" ) == 0 )
    {
        if ( numParts >= 3 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            uint16_t threshold = (uint16_t)strtoul( parts[2], NULL, 10 );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetLightThreshold( addr, threshold );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'lightthreshold' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: lightthreshold <addr> <value>\n" );
        }
    }
    else if ( strcmp( base, "onicsdelay" ) == 0 )
    {
        if ( numParts >= 3 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            uint16_t delayMs = (uint16_t)strtoul( parts[2], NULL, 10 );
            bool found = false;
#if ENABLE_ONICS_BUTTON
            if ( OnicsButton_IsKnown( addr ) )
            {
                printf( "Writing ButtonPressActionDelay (0x8001) = %d ms to Onics 0x%04X...\n", delayMs, addr );
                uint8_t payloadFull[8];
                static uint8_t s_cliSeq = 0;
                payloadFull[0] = 0x00;
                payloadFull[1] = s_cliSeq++;
                payloadFull[2] = 0x02;
                payloadFull[3] = 0x01;
                payloadFull[4] = 0x80;
                payloadFull[5] = 0x21; // Uint16
                payloadFull[6] = delayMs & 0xFF;
                payloadFull[7] = (delayMs >> 8) & 0xFF;

                ZNP_AfDataRequestExt( 2, addr, 0x20, 0, 8, 0x0006, 0, 0, 15, payloadFull, 8 );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support the 'onicsdelay' command.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: onicsdelay <addr hex> <delay_in_ms>\n" );
        }
    }
    else if ( strcmp( base, "discover" ) == 0 )
    {
        if ( numParts >= 2 )
        {
            uint16_t addr = (uint16_t)strtol( parts[1], NULL, 16 );
            printf( "Actively enumerating endpoints on 0x%04X...\n", addr );
            ZNP_ZdoActiveEpReq( addr );
            ZNP_QuerySimpleDesc( addr, 43 );
            ZNP_QuerySimpleDesc( addr, 1 );
        }
        else
        {
            printf( "Broadcasting discovery requests...\n" );
            uint16_t sirenIn = 0x0502;
            ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 1, &sirenIn, 0, NULL );
            uint16_t btnIn = 0x0500;
            ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 1, &btnIn, 0, NULL );
            uint16_t occIn = 0x0406;
            ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 1, &occIn, 0, NULL );

#if ENABLE_SIREN
            Siren_DiscoverAllActiveEp();
#endif
#if ENABLE_OKOS_SIREN
            OkosSiren_DiscoverAllActiveEp();
#endif
#if ENABLE_AQARA_BUTTON
            AqaraButton_DiscoverAllActiveEp();
#endif
#if ENABLE_ONICS_BUTTON
            OnicsButton_DiscoverAllActiveEp();
#endif
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_DiscoverAllActiveEp();
#endif
        }
    }
    else if ( strcmp( base, "env" ) == 0 )
    {
        if ( numParts >= 2 )
        {
            uint16_t addr = (uint16_t)strtol( parts[1], NULL, 16 );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_ReadEnvironment( addr );
                found = true;
            }
#endif
#if ENABLE_VIBRATION_SENSOR
            if (VibrationSensor_IsKnown(addr)) {
                VibrationSensor_ReadEnvironment( addr );
                found = true;
            }
#endif
#if ENABLE_CONTACT_SENSOR
            if (ContactSensor_IsKnown(addr)) {
                ContactSensor_ReadEnvironment( addr );
                found = true;
            }
#endif
#if ENABLE_ONICS_BUTTON
            if (OnicsButton_IsKnown(addr)) {
                OnicsButton_ReadEnvironment( addr );
                found = true;
            }
#endif
#if ENABLE_SIREN
            if (Siren_GetEndpoint(addr) != 0) {
                Siren_ReadEnvironment( addr );
                found = true;
            }
#endif
#if ENABLE_OKOS_SIREN
            if (OkosSiren_IsKnown(addr)) {
                OkosSiren_ReadEnvironment( addr );
                OkosSiren_ReadBattery( addr );
                found = true;
            }
#endif
#if ENABLE_AQARA_TVOC
            if (AqaraTvoc_IsKnown(addr)) {
                AqaraTvoc_ReadEnvironment( addr );
                found = true;
            }
#endif
            if (!found) {
                const char* devName = Device_GetName(addr);
                if (strcmp(devName, "Unknown Device") == 0) {
                    printf("Error: Address 0x%04X is not registered in the network.\n", addr);
                } else {
                    printf("Error: The device at 0x%04X (%s) does not support environment queries.\n", addr, devName);
                }
            }
        }
        else
        {
            printf( "Usage: env <shortAddr>\n" );
        }
    }
    else if ( strcmp( base, "permit" ) == 0 )
    {
        int duration = 60;
        if ( numParts > 1 )
        {
            duration = atoi( parts[1] );
        }
        printf( "Opening permit join for %d seconds...\n", duration );
        ZNP_PermitJoin( duration );
    }
    else if ( strcmp( base, "rebind" ) == 0 )
    {
        if ( numParts == 2 )
        {
            uint16_t addr = (uint16_t)strtoul( parts[1], NULL, 16 );
            
            pthread_mutex_lock( &g_deviceMutex );
            bool found = false;
#if ENABLE_AQARA_OCCUPANCY
            for ( int i = 0; i < g_numAqaraOccupancies; i++ ) {
                if ( g_aqaraOccupancies[i].shortAddr == addr ) {
                    g_aqaraOccupancies[i].configured = false;
                    found = true;
                    break;
                }
            }
#endif
#if ENABLE_CONTACT_SENSOR
            for ( int i = 0; i < g_numContactSensors; i++ ) {
                if ( g_contactSensors[i].shortAddr == addr ) {
                    g_contactSensors[i].configured = false;
                    found = true;
                    break;
                }
            }
#endif
#if ENABLE_VIBRATION_SENSOR
            for ( int i = 0; i < g_numVibrationSensors; i++ ) {
                if ( g_vibrationSensors[i].shortAddr == addr ) {
                    g_vibrationSensors[i].configured = false;
                    found = true;
                    break;
                }
            }
#endif
#if ENABLE_AQARA_BUTTON
            for ( int i = 0; i < g_numAqaraButtons; i++ ) {
                if ( g_aqaraButtons[i].shortAddr == addr ) {
                    g_aqaraButtons[i].configured = false;
                    found = true;
                    break;
                }
            }
#endif
#if ENABLE_ONICS_BUTTON
            for ( int i = 0; i < g_numOnicsButtons; i++ ) {
                if ( g_onicsButtons[i].shortAddr == addr ) {
                    g_onicsButtons[i].configured = false;
                    found = true;
                    break;
                }
            }
#endif
#if ENABLE_SIREN
            for ( int i = 0; i < g_numSirens; i++ ) {
                if ( g_sirens[i].shortAddr == addr ) {
                    g_sirens[i].configured = false;
                    found = true;
                    break;
                }
            }
#endif
            pthread_mutex_unlock( &g_deviceMutex );

            if ( found )
            {
                printf("✅ SUCCESS: Forced reconfiguration for 0x%04X.\n", addr);
#if ENABLE_AQARA_OCCUPANCY
                AqaraOccupancy_PostAssign( addr );
#endif
#if ENABLE_CONTACT_SENSOR
                ContactSensor_PostAssign( addr );
#endif
#if ENABLE_VIBRATION_SENSOR
                VibrationSensor_PostAssign( addr );
#endif
#if ENABLE_AQARA_BUTTON
                AqaraButton_PostAssign( addr );
#endif
#if ENABLE_ONICS_BUTTON
                OnicsButton_PostAssign( addr );
#endif
#if ENABLE_SIREN
                Siren_PostAssign( addr );
#endif
#if ENABLE_OKOS_SIREN
                OkosSiren_PostAssign( addr );
#endif
            }
            else
            {
                printf("❌ ERROR: Device 0x%04X not found in any registry.\n", addr);
            }
        }
        else
        {
            printf("❌ ERROR: Usage: rebind <shortAddr>\n");
        }
    }
    else if ( strcmp( base, "remove" ) == 0 )
    {
        if ( numParts >= 2 )
        {
            uint16_t addr = (uint16_t)strtol( parts[1], NULL, 16 );
            uint8_t ieee[8];
            bool hasIeee = Device_GetDiscoveredIeee( addr, ieee );
            if (hasIeee) {
                printf( "Sending network leave request to 0x%04X (IEEE: %02X%02X%02X%02X%02X%02X%02X%02X)...\n", addr,
                        ieee[7], ieee[6], ieee[5], ieee[4], ieee[3], ieee[2], ieee[1], ieee[0] );
            } else {
                printf( "Sending network leave request to 0x%04X (No IEEE known, using short address only)...\n", addr );
            }
            ZNP_ZdoMgmtLeaveReq( addr, hasIeee ? ieee : NULL, false, false );
            printf( "Note: To fully clear from memory, you may still need to delete its line from devices.txt and restart.\n" );
        }
        else
        {
            printf( "Usage: remove <shortAddr>\n" );
        }
    }
    else if ( strcmp( base, "exit" ) == 0 || strcmp( base, "quit" ) == 0 )
    {
        printf( "Exiting...\n" );
        ZNP_Close();
        exit( 0 );
    }
    else
    {
        printf( "Unknown command: '%s'. Type 'help' for commands.\n", base );
    }
}

static void *Cli_Thread( void *arg_ )
{
    (void)arg_;
    char line[256];
    while ( 1 )
    {
        if ( fgets( line, sizeof( line ), stdin ) != NULL )
        {
            int len = strlen( line );
            while ( len > 0 && ( line[len - 1] == '\n' || line[len - 1] == '\r' ) )
            {
                line[len - 1] = '\0';
                len--;
            }
            Cli_HandleCommand( line );
        }
        else
        {
            if ( feof( stdin ) ) break;
            usleep( 100000 );
        }
    }
    return NULL;
}

void Cli_Start(void)
{
    pthread_t cliThread;
    if ( pthread_create( &cliThread, NULL, Cli_Thread, NULL ) != 0 )
    {
        perror( "Failed to create CLI thread" );
    }
}
