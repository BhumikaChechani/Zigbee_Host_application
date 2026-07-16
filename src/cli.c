#include "cli.h"
#include "znp_host.h"
#include "config.h"

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
        printf( "  siren on                       - Turn all sirens ON\n" );
        printf( "  siren off                      - Turn all sirens OFF\n" );
        printf( "  siren vol <0-3>                - Set siren volume globally (0=low, 3=very high)\n" );
        printf( "  siren mode <1-6>               - Set siren sound mode globally\n" );
        printf( "                                   * 1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic\n" );
        printf( "  siren test <addr> [mode]       - Test siren warning directly\n" );
        printf( "  siren stop <addr>              - Stop a specific siren\n" );
        printf( "\n--- Sensor Configuration ---\n" );
        printf( "  env <addr>                     - Fetch environment data (Temp/Humidity/Battery)\n" );
        printf( "                                   * Works for: Aqara Occupancy, Frient Vibration, Frient Siren\n" );
        printf( "  sensitivity <addr> <level>     - Set physical sensitivity level\n" );
        printf( "                                   * Aqara Occupancy: 1=Low, 2=Medium, 3=High\n" );
        printf( "                                   * Frient Vibration: 1=Most sensitive ... 15=Least sensitive (Default 10)\n" );
        printf( "  zone <addr> <id> <min> <max>   - Set detection zone distances (cm)\n" );
        printf( "                                   * Works for: Aqara Occupancy\n" );
        printf( "  zonedel <addr> <id>            - Delete a detection zone (Aqara Occupancy)\n" );
        printf( "  spatiallearn <addr>            - Trigger AI Spatial Learning (Aqara Occupancy)\n" );
        printf( "  forcesetup <addr>              - Force re-bind and config payload to sensor\n" );
        printf( "  onicsdelay <addr> <ms>         - Write ButtonPressActionDelay (attr 0x8001)\n" );
        printf( "  lightthreshold [value]         - Set or show light threshold for Aqara Occupancy\n" );
        printf( "  exit                           - Quit application\n\n" );
    }
    else if ( strcmp( base, "status" ) == 0 )
    {
        pthread_mutex_lock( &g_deviceMutex );
        printf( "\n--- System Status ---\n" );
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
#endif
#if ENABLE_VIBRATION_SENSOR
        VibrationSensor_PrintStatus();
#endif
#if ENABLE_AQARA_OCCUPANCY
        AqaraOccupancy_PrintStatus();
#endif
        printf( "\n" );
    }
    else if ( strcmp( base, "siren" ) == 0 )
    {
        if ( numParts < 2 )
        {
            printf( "Usage: siren [on|off|test|vol]\n" );
            return;
        }
        if ( strcmp( parts[1], "on" ) == 0 )
        {
            printf( "Manually starting all sirens...\n" );
#if ENABLE_SIREN
            Siren_ControlAll( 1 );
#endif
        }
        else if ( strcmp( parts[1], "off" ) == 0 )
        {
            printf( "Manually stopping all sirens...\n" );
#if ENABLE_SIREN
            Siren_ControlAll( 0 );
#endif
        }
        else if ( strcmp( parts[1], "vol" ) == 0 )
        {
            if ( numParts < 3 )
            {
                printf( "Usage: siren vol <0-3> (0=low, 1=medium, 2=high, 3=very high)\n" );
                return;
            }
            uint8_t vol = (uint8_t)strtoul( parts[2], NULL, 10 );
#if ENABLE_SIREN
            Siren_SetVolume( vol );
#endif
        }
        else if ( strcmp( parts[1], "mode" ) == 0 )
        {
            if ( numParts < 3 )
            {
                printf( "Usage: siren mode <1-6> (1=Burglar, 2=Fire, 3=Emergency, 4=Police Panic, 5=Fire Panic, 6=Emergency Panic)\n" );
                return;
            }
            uint8_t mode = (uint8_t)strtoul( parts[2], NULL, 10 );
#if ENABLE_SIREN
            Siren_SetMode( mode );
#endif
        }
        else if ( strcmp( parts[1], "test" ) == 0 )
        {
            if ( numParts < 3 )
            {
                printf( "Usage: siren test <addr_hex> [mode]\n" );
                return;
            }
            uint16_t addr = (uint16_t)strtol( parts[2], NULL, 16 );
            
#if ENABLE_SIREN
            uint8_t ep = Siren_GetEndpoint(addr);
            if (ep == 0)
            {
                printf("Error: Siren 0x%04X is not registered. Run 'status' or trigger discovery.\n", addr);
                return;
            }
            uint8_t testMode = (numParts >= 4) ? (uint8_t)strtol( parts[3], NULL, 10 ) : Siren_GetMode();
            ZNP_SendSirenWarning( addr, ep, 0xBB, testMode, Siren_GetVolume(), 240 );
#endif
        }
        else if ( strcmp( parts[1], "stop" ) == 0 )
        {
            if ( numParts < 3 )
            {
                printf( "Usage: siren stop <addr_hex>\n" );
                return;
            }
            uint16_t addr = (uint16_t)strtol( parts[2], NULL, 16 );
            
#if ENABLE_SIREN
            uint8_t ep = Siren_GetEndpoint(addr);
            if (ep == 0)
            {
                printf("Error: Siren 0x%04X is not registered. Run 'status' or trigger discovery.\n", addr);
                return;
            }
            ZNP_SendSirenWarning( addr, ep, 0xBB, 0, Siren_GetVolume(), 240 );
#endif
        }
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
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_SetZone( addr, zoneIdx, minCm, maxCm );
#endif
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
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_DeleteZone( addr, zoneIdx );
#endif
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
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_SetSensitivity( addr, level );
            }
#endif
#if ENABLE_VIBRATION_SENSOR
            if (VibrationSensor_IsKnown(addr)) {
                VibrationSensor_SetSensitivity( addr, level );
            }
#endif
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
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_SpatialLearning( addr );
#endif
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
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_PostAssign( addr );
#endif
        }
        else
        {
            printf( "Usage: forcesetup <addr hex>\n" );
        }
    }
    else if ( strcmp( base, "lightthreshold" ) == 0 )
    {
        if ( numParts >= 2 )
        {
            uint16_t threshold = (uint16_t)strtoul( parts[1], NULL, 10 );
#if ENABLE_AQARA_OCCUPANCY
            AqaraOccupancy_SetLightThreshold( threshold );
            printf( "Aqara Occupancy light intensity threshold set to %u\n", threshold );
#else
            (void)threshold;
#endif
        }
        else
        {
#if ENABLE_AQARA_OCCUPANCY
            printf( "Current Aqara Occupancy light intensity threshold: %u\n", AqaraOccupancy_GetLightThreshold() );
#endif
            printf( "Usage: lightthreshold <value>\n" );
        }
    }
    else if ( strcmp( base, "onicsdelay" ) == 0 )
    {
        if ( numParts >= 3 )
        {
            uint16_t addr = strtoul( parts[1], NULL, 16 );
            uint16_t delayMs = (uint16_t)strtoul( parts[2], NULL, 10 );
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
            }
            else
            {
                printf( "Error: 0x%04X is not a known Onics Button.\n", addr );
            }
#endif
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
#if ENABLE_AQARA_OCCUPANCY
            if (AqaraOccupancy_IsKnown(addr)) {
                AqaraOccupancy_ReadEnvironment( addr );
            }
#endif
#if ENABLE_VIBRATION_SENSOR
            if (VibrationSensor_IsKnown(addr)) {
                VibrationSensor_ReadEnvironment( addr );
            }
#endif
#if ENABLE_CONTACT_SENSOR
            if (ContactSensor_IsKnown(addr)) {
                ContactSensor_ReadEnvironment( addr );
            }
#endif
#if ENABLE_ONICS_BUTTON
            if (OnicsButton_IsKnown(addr)) {
                OnicsButton_ReadEnvironment( addr );
            }
#endif
#if ENABLE_SIREN
            if (Siren_GetEndpoint(addr) != 0) {
                Siren_ReadEnvironment( addr );
            }
#endif
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
