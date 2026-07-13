///
/// @file   main.c
/// @brief  Startup sequence, the incoming-frame dispatcher, device persistence,
///         and the interactive CLI for the ZNP host controller.
///
/// main() brings the coordinator up, then the main loop acts as the dispatcher:
/// it drains the AREQ event queue, classifies devices from ZDO responses, and
/// routes AF messages to the owning sensor's worker thread. Sensor device I/O
/// and the siren policy run on their own threads.
///
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

#include "usecase.h"
#include "sensor_common.h"
#include "cli.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

// Shared mutex protecting the global registries
pthread_mutex_t g_deviceMutex;

// Device registry persistence file. Relative to the current working directory
// (run the program from its own folder).
static const char DEVICES_FILE[] = "devices.txt";

// Local (static) function declarations
static void Main_HandleIncomingFrame( const MT_FRAME_T *frame_ ); ///< Route one AREQ frame.


void Device_AddDiscoveredIeee( uint16_t shortAddr_, const uint8_t *ieee_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numDiscoveredIeees; i++ )
    {
        if ( g_discoveredIeees[i].shortAddr == shortAddr_ )
        {
            memcpy( g_discoveredIeees[i].ieee, ieee_, 8 );
            pthread_mutex_unlock( &g_deviceMutex );
            return;
        }
    }
    if ( g_numDiscoveredIeees < MAX_DISCOVERED_IEEES )
    {
        g_discoveredIeees[g_numDiscoveredIeees].shortAddr = shortAddr_;
        memcpy( g_discoveredIeees[g_numDiscoveredIeees].ieee, ieee_, 8 );
        g_numDiscoveredIeees++;
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

bool Device_GetDiscoveredIeee( uint16_t shortAddr_, uint8_t *ieeeOut_ )
{
    bool found = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numDiscoveredIeees; i++ )
    {
        if ( g_discoveredIeees[i].shortAddr == shortAddr_ )
        {
            if ( ieeeOut_ != NULL )
            {
                memcpy( ieeeOut_, g_discoveredIeees[i].ieee, 8 );
            }
            found = true;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return found;
}

#define MAX_PENDING_DISCOVERIES 32
typedef struct
{
    uint16_t shortAddr;
    double lastQueryTime;
} PENDING_DISCOVERY_T;

static PENDING_DISCOVERY_T s_pendingDiscoveries[MAX_PENDING_DISCOVERIES];
static int s_numPendingDiscoveries = 0;

///
/// @brief  Rate-limit descriptor queries for an unknown device to once per 30 s.
///
/// Prevents a chatty unknown device from triggering a flood of Simple_Desc /
/// discovery requests while its first query is still in flight.
///
/// @param  shortAddr_  Device network address.
/// @return true if the caller should issue a (re)query now; false if throttled.
///
static bool Device_ShouldQuery( uint16_t shortAddr_ )
{
    double now = ZNP_GetCurrentTime();
    bool result = true;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < s_numPendingDiscoveries; i++ )
    {
        if ( s_pendingDiscoveries[i].shortAddr == shortAddr_ )
        {
            if ( now - s_pendingDiscoveries[i].lastQueryTime < 30.0 )
            {
                result = false;
            }
            else
            {
                s_pendingDiscoveries[i].lastQueryTime = now;
            }
            pthread_mutex_unlock( &g_deviceMutex );
            return result;
        }
    }

    if ( s_numPendingDiscoveries < MAX_PENDING_DISCOVERIES )
    {
        s_pendingDiscoveries[s_numPendingDiscoveries].shortAddr = shortAddr_;
        s_pendingDiscoveries[s_numPendingDiscoveries].lastQueryTime = now;
        s_numPendingDiscoveries++;
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return true;
}

void Device_Save( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    FILE *file = fopen( DEVICES_FILE, "w" );
    if ( !file )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }

#if ENABLE_SIREN
    for ( int i = 0; i < g_numSirens; i++ )
    {
        fprintf( file, "siren %04X %02X ", g_sirens[i].shortAddr, g_sirens[i].endpoint );
        for ( int j = 0; j < 8; j++ )
        {
            fprintf( file, "%02X", g_sirens[i].ieee[j] );
        }
        fprintf( file, " %d %d\n", g_sirens[i].hasIeee ? 1 : 0, g_sirens[i].zoneId );
    }
#endif

#if ENABLE_AQARA_BUTTON
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        fprintf( file, "aqara %04X %02X ", g_aqaraButtons[i].shortAddr, g_aqaraButtons[i].endpoint );
        for ( int j = 0; j < 8; j++ )
        {
            fprintf( file, "%02X", g_aqaraButtons[i].ieee[j] );
        }
        fprintf( file, " %d\n", g_aqaraButtons[i].hasIeee ? 1 : 0 );
    }
#endif

#if ENABLE_ONICS_BUTTON
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        fprintf( file, "onics %04X %02X ", g_onicsButtons[i].shortAddr, g_onicsButtons[i].endpoint );
        for ( int j = 0; j < 8; j++ )
        {
            fprintf( file, "%02X", g_onicsButtons[i].ieee[j] );
        }
        fprintf( file, " %d %d\n", g_onicsButtons[i].hasIeee ? 1 : 0, g_onicsButtons[i].zoneId );
    }
#endif

#if ENABLE_AQARA_OCCUPANCY
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        fprintf( file, "occupancy %04X %02X ", g_aqaraOccupancies[i].shortAddr, g_aqaraOccupancies[i].endpoint );
        for ( int j = 0; j < 8; j++ )
        {
            fprintf( file, "%02X", g_aqaraOccupancies[i].ieee[j] );
        }
        fprintf( file, " %d", g_aqaraOccupancies[i].hasIeee ? 1 : 0 );
        for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
        {
            fprintf( file, " %d %u %u", g_aqaraOccupancies[i].zones[z].isActive ? 1 : 0, 
                     g_aqaraOccupancies[i].zones[z].minCm, g_aqaraOccupancies[i].zones[z].maxCm );
        }
        fprintf( file, "\n" );
    }
#endif

#if ENABLE_CONTACT_SENSOR
    for ( int i = 0; i < g_numContactSensors; i++ )
    {
        fprintf( file, "contact %04X %02X ", g_contactSensors[i].shortAddr, g_contactSensors[i].endpoint );
        for ( int j = 0; j < 8; j++ )
        {
            fprintf( file, "%02X", g_contactSensors[i].ieee[j] );
        }
        fprintf( file, " %d %d\n", g_contactSensors[i].hasIeee ? 1 : 0, g_contactSensors[i].zoneId );
    }
#endif

#if ENABLE_VIBRATION_SENSOR
    for ( int i = 0; i < g_numVibrationSensors; i++ )
    {
        fprintf( file, "vibration %04X %02X ", g_vibrationSensors[i].shortAddr, g_vibrationSensors[i].endpoint );
        for ( int j = 0; j < 8; j++ )
        {
            fprintf( file, "%02X", g_vibrationSensors[i].ieee[j] );
        }
        fprintf( file, " %d %d %d\n", g_vibrationSensors[i].hasIeee ? 1 : 0, g_vibrationSensors[i].zoneId, g_vibrationSensors[i].sensitivity );
    }
#endif

    fclose( file );
    pthread_mutex_unlock( &g_deviceMutex );
}

void Device_Load( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    FILE *file = fopen( DEVICES_FILE, "r" );
    if ( !file )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }

    char type[32];
    unsigned int shortAddr;
    unsigned int endpoint;
    unsigned int hasIeee;
    unsigned int zoneId;
    char ieeeStr[32];

    while ( fscanf( file, "%31s %X %X %31s", type, &shortAddr, &endpoint, ieeeStr ) == 4 )
    {
        uint8_t ieee[8];
        for ( int i = 0; i < 8; i++ )
        {
            unsigned int byteVal;
            sscanf( &ieeeStr[i * 2], "%2X", &byteVal );
            ieee[i] = (uint8_t)byteVal;
        }

        if ( 0 )
        {
        }
#if ENABLE_SIREN
        else if ( strcmp( type, "siren" ) == 0 )
        {
            int items = fscanf( file, "%u %u", &hasIeee, &zoneId );
            if ( items == 1 )
            {
                zoneId = 0;
            }
            if ( items >= 1 && g_numSirens < MAX_SIRENS )
            {
                g_sirens[g_numSirens].shortAddr = shortAddr;
                g_sirens[g_numSirens].endpoint = endpoint;
                g_sirens[g_numSirens].lastSeen = ZNP_GetCurrentTime();
                g_sirens[g_numSirens].hasIeee = ( hasIeee != 0 );
                memcpy( g_sirens[g_numSirens].ieee, ieee, 8 );
                g_sirens[g_numSirens].zoneId = zoneId;
                g_sirens[g_numSirens].configured = true;
                g_numSirens++;
                Device_AddDiscoveredIeee( shortAddr, ieee );
            }
        }
#endif
#if ENABLE_AQARA_BUTTON
        else if ( strcmp( type, "aqara" ) == 0 )
        {
            if ( fscanf( file, "%u", &hasIeee ) == 1 && g_numAqaraButtons < MAX_AQARA_BUTTONS )
            {
                g_aqaraButtons[g_numAqaraButtons].shortAddr = shortAddr;
                g_aqaraButtons[g_numAqaraButtons].endpoint = endpoint;
                g_aqaraButtons[g_numAqaraButtons].lastSeen = ZNP_GetCurrentTime();
                g_aqaraButtons[g_numAqaraButtons].hasIeee = ( hasIeee != 0 );
                memcpy( g_aqaraButtons[g_numAqaraButtons].ieee, ieee, 8 );
                g_aqaraButtons[g_numAqaraButtons].configured = true;
                g_numAqaraButtons++;
                Device_AddDiscoveredIeee( shortAddr, ieee );
            }
        }
#endif
#if ENABLE_ONICS_BUTTON
        else if ( strcmp( type, "onics" ) == 0 )
        {
            if ( fscanf( file, "%u %u", &hasIeee, &zoneId ) == 2 && g_numOnicsButtons < MAX_ONICS_BUTTONS )
            {
                g_onicsButtons[g_numOnicsButtons].shortAddr = shortAddr;
                g_onicsButtons[g_numOnicsButtons].endpoint = endpoint;
                g_onicsButtons[g_numOnicsButtons].lastSeen = ZNP_GetCurrentTime();
                g_onicsButtons[g_numOnicsButtons].hasIeee = ( hasIeee != 0 );
                memcpy( g_onicsButtons[g_numOnicsButtons].ieee, ieee, 8 );
                g_onicsButtons[g_numOnicsButtons].zoneId = zoneId;
                g_onicsButtons[g_numOnicsButtons].configured = true;
                g_numOnicsButtons++;
                Device_AddDiscoveredIeee( shortAddr, ieee );
            }
        }
#endif
#if ENABLE_AQARA_OCCUPANCY
        else if ( strcmp( type, "occupancy" ) == 0 )
        {
            // Default values in case fscanf fails to read them (legacy format)
            int zActive[MAX_OCCUPANCY_ZONES] = { 1, 0, 0, 0 };
            uint32_t zMin[MAX_OCCUPANCY_ZONES] = { 0, 0, 0, 0 };
            uint32_t zMax[MAX_OCCUPANCY_ZONES] = { 600, 0, 0, 0 };
            
            int scanned = fscanf( file, "%u", &hasIeee );
            if ( scanned >= 1 && g_numAqaraOccupancies < MAX_AQARA_OCCUPANCY )
            {
                // Try to read the zones array
                for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
                {
                    if ( fscanf( file, "%d %u %u", &zActive[z], &zMin[z], &zMax[z] ) != 3 ) break;
                }
                
                g_aqaraOccupancies[g_numAqaraOccupancies].shortAddr = shortAddr;
                g_aqaraOccupancies[g_numAqaraOccupancies].endpoint = endpoint;
                g_aqaraOccupancies[g_numAqaraOccupancies].lastSeen = ZNP_GetCurrentTime();
                g_aqaraOccupancies[g_numAqaraOccupancies].hasIeee = ( hasIeee != 0 );
                memcpy( g_aqaraOccupancies[g_numAqaraOccupancies].ieee, ieee, 8 );
                
                for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
                {
                    g_aqaraOccupancies[g_numAqaraOccupancies].zones[z].isActive = ( zActive[z] != 0 );
                    g_aqaraOccupancies[g_numAqaraOccupancies].zones[z].minCm = zMin[z];
                    g_aqaraOccupancies[g_numAqaraOccupancies].zones[z].maxCm = zMax[z];
                }
                
                g_aqaraOccupancies[g_numAqaraOccupancies].configured = true;
                g_numAqaraOccupancies++;
                Device_AddDiscoveredIeee( shortAddr, ieee );
            }
        }
#endif
#if ENABLE_CONTACT_SENSOR
        else if ( strcmp( type, "contact" ) == 0 )
        {
            if ( fscanf( file, "%u %u", &hasIeee, &zoneId ) == 2 && g_numContactSensors < MAX_CONTACT_SENSORS )
            {
                g_contactSensors[g_numContactSensors].shortAddr = shortAddr;
                g_contactSensors[g_numContactSensors].endpoint = endpoint;
                g_contactSensors[g_numContactSensors].lastSeen = ZNP_GetCurrentTime();
                g_contactSensors[g_numContactSensors].hasIeee = ( hasIeee != 0 );
                memcpy( g_contactSensors[g_numContactSensors].ieee, ieee, 8 );
                g_contactSensors[g_numContactSensors].zoneId = zoneId;
                g_contactSensors[g_numContactSensors].configured = true;
                g_numContactSensors++;
                Device_AddDiscoveredIeee( shortAddr, ieee );
            }
        }
#endif
#if ENABLE_VIBRATION_SENSOR
        else if ( strcmp( type, "vibration" ) == 0 )
        {
            unsigned int sens = 10;
            int scanned = fscanf( file, "%u %u %u", &hasIeee, &zoneId, &sens );
            if ( scanned >= 2 && g_numVibrationSensors < MAX_VIBRATION_SENSORS )
            {
                g_vibrationSensors[g_numVibrationSensors].shortAddr = shortAddr;
                g_vibrationSensors[g_numVibrationSensors].endpoint = endpoint;
                g_vibrationSensors[g_numVibrationSensors].lastSeen = ZNP_GetCurrentTime();
                g_vibrationSensors[g_numVibrationSensors].hasIeee = ( hasIeee != 0 );
                memcpy( g_vibrationSensors[g_numVibrationSensors].ieee, ieee, 8 );
                g_vibrationSensors[g_numVibrationSensors].zoneId = zoneId;
                g_vibrationSensors[g_numVibrationSensors].sensitivity = sens;
                g_vibrationSensors[g_numVibrationSensors].configured = true;
                g_vibrationSensors[g_numVibrationSensors].isVibrating = false;
                g_vibrationSensors[g_numVibrationSensors].lastVibrationTime = 0.0;
                g_vibrationSensors[g_numVibrationSensors].isMoving = false;
                g_vibrationSensors[g_numVibrationSensors].lastMovementTime = 0.0;
                g_numVibrationSensors++;
                Device_AddDiscoveredIeee( shortAddr, ieee );
            }
        }
#endif
    }

    fclose( file );
    printf( "📋 Loaded existing devices from devices.txt\n" );
    pthread_mutex_unlock( &g_deviceMutex );
}

///
/// @brief  Program entry point: parse args, bring up the coordinator, run loop.
///
/// Initializes modules, loads persisted devices, opens the serial port, runs
/// the ZNP startup sequence (ping, optional factory-new, network start,
/// endpoint + callback registration, permit join, discovery), starts the
/// worker threads, then enters the dispatcher loop.
///
/// @param  argc  Argument count.
/// @param  argv  Arguments: optional serial port path and/or -f/--factory-new.
/// @return 0 on normal exit; 1 on a fatal startup error.
///
int main( int argc, char *argv[] )
{
    setvbuf( stdin, NULL, _IOLBF, 0 );
    setvbuf( stdout, NULL, _IOLBF, 0 );

    // Initialize g_deviceMutex as a recursive mutex
    pthread_mutexattr_t attr;
    pthread_mutexattr_init( &attr );
    pthread_mutexattr_settype( &attr, PTHREAD_MUTEX_RECURSIVE );
    pthread_mutex_init( &g_deviceMutex, &attr );
    pthread_mutexattr_destroy( &attr );

    // Initialize the use-case layer and each sensor module (lists + inboxes).
    UseCase_Init();
#if ENABLE_SIREN
    Siren_Init();
#endif
#if ENABLE_AQARA_BUTTON
    AqaraButton_Init();
#endif
#if ENABLE_ONICS_BUTTON
    OnicsButton_Init();
#endif
#if ENABLE_AQARA_OCCUPANCY
    AqaraOccupancy_Init();
#endif
#if ENABLE_CONTACT_SENSOR
    ContactSensor_Init();
#endif
#if ENABLE_VIBRATION_SENSOR
    VibrationSensor_Init();
#endif

    // Load persisted devices
    Device_Load();

    const char *port = PORT_DEFAULT;
    bool forceFactoryNew = false;
    for ( int i = 1; i < argc; i++ )
    {
        if ( strcmp( argv[i], "-f" ) == 0 || strcmp( argv[i], "--factory-new" ) == 0 )
        {
            forceFactoryNew = true;
        }
        else if ( argv[i][0] != '-' )
        {
            port = argv[i];
        }
    }

    printf( "\n============================================================\n" );
    printf( "  ZNP MT Host Controller for Siren & Button Integration (C Version)\n" );
    printf( "  Port: %s  (Factory New: %s)\n", port, forceFactoryNew ? "YES" : "NO" );
    printf( "============================================================\n\n" );

    if ( !ZNP_Init( port ) )
    {
        printf( "❌ Failed to open serial port %s\n", port );
        return 1;
    }

    sleep( 1 );

    // 1. Ping coordinator
    printf( "[1] SYS_PING...\n" );
    if ( !ZNP_SysPing() )
    {
        printf( "❌ No response from ZNP. Check port and connections.\n" );
        ZNP_Close();
        return 1;
    }
    printf( "  ✅ ZNP responsive\n\n" );

    // 2. Get device info
    printf( "[2] UTIL_GET_DEVICE_INFO...\n" );
    int state = ZNP_UtilGetDeviceInfo();
    printf( "\n" );

    if ( state == 9 && !forceFactoryNew )
    {
        printf( "  Coordinator already active — skipping startup.\n\n" );
    }
    else
    {
        if ( forceFactoryNew )
        {
            printf( "[3] Factory-new sequence (clear NV + write config)...\n" );
            ZNP_FactoryNew();
            printf( "\n" );
        }
        else
        {
            printf( "[3] SYS_RESET_REQ (soft reset)...\n" );
            ZNP_SysResetReq( false );
            sleep( 1 );
            printf( "\n" );
        }

        printf( "[4] ZDO_STARTUP_FROM_APP...\n" );
        ZNP_ZdoStartupFromApp( 100 );
        printf( "\n" );

        printf( "[5] Waiting for coordinator state (state=9)...\n" );
        bool active = false;
        double deadline = ZNP_GetCurrentTime() + 30.0;
        MT_FRAME_T rx;
        while ( ZNP_GetCurrentTime() < deadline )
        {
            if ( EventQueue_Pop( &g_eventQueue, &rx, 100 ) )
            {
                // ZDO_STATE_CHANGE_IND: cmd0=0x45, cmd1=0xC0
                if ( rx.cmd0 == 0x45 && rx.cmd1 == 0xC0 && rx.len >= 1 )
                {
                    uint8_t devState = rx.payload[0];
                    printf( "  -> State: %d\n", devState );
                    if ( devState == 9 )
                    {
                        printf( "  ✅ COORDINATOR ACTIVE!\n" );
                        active = true;
                        break;
                    }
                }
                else
                {
                    // Let Main_HandleIncomingFrame process other events (joins etc.)
                    Main_HandleIncomingFrame( &rx );
                }
            }
        }
        printf( "\n" );

        if ( !active )
        {
            printf( "[5b] Re-checking device info...\n" );
            state = ZNP_UtilGetDeviceInfo();
            printf( "\n" );
            if ( state != 9 )
            {
                printf( "❌ Coordinator did not start.\n" );
                ZNP_Close();
                return 1;
            }
        }
    }

    // 5.5 Register endpoint and ZDO callbacks.
    // Input clusters must include everything our devices REPORT on, or the ZNP
    // won't route those reports to us (and devices won't bind for reporting to a
    // cluster the coordinator doesn't advertise). 0xFCC0 = Aqara manufacturer
    // cluster (occupancy/presence attr 0x0142), 0x0012 = Multistate Input.
    printf( "[5.5] Registering Application Endpoint and ZDO Callbacks...\n" );
    uint16_t inClusters[8] = { 0x0000, 0x0003, 0x0004, 0x0005, 0x0006, 0x0406, 0xFCC0, 0x0012 };
    uint16_t outClusters[7] = { 0x0500, 0x0502, 0x0406, 0xFCC0, 0x0402, 0x0405, 0x0400 };
    ZNP_AfRegister( 8, 0x0104, 0x0007, 1, 0, 8, inClusters, 7, outClusters );

    ZNP_ZdoMsgCbRegister( 0x8001 ); // IEEE_addr_rsp
    ZNP_ZdoMsgCbRegister( 0x8004 ); // Simple_desc_rsp
    ZNP_ZdoMsgCbRegister( 0x8005 ); // Active_EP_rsp
    ZNP_ZdoMsgCbRegister( 0x8006 ); // Match_desc_rsp
    ZNP_ZdoMsgCbRegister( 0x0013 ); // Device_annce
    printf( "\n" );

    // Turn Red LED OFF initially


    // 5.7 Relax the Trust Center key-exchange policy. Zigbee 3.0 defaults to
    // kicking any joiner that does not upgrade its TC link key within a few
    // seconds; many Aqara sensors never do, so they rejoin endlessly. Allowing
    // them to stay on the global key lets them settle on the network.
    printf( "[5.7] Relaxing Trust Center key-exchange policy...\n" );
    ZNP_BdbSetTcRequireKeyExchange( false );
    printf( "\n" );

    // 6. Open permit join
    printf( "[6] Opening permit join (all methods)...\n" );
    ZNP_PermitJoin( PERMIT_JOIN_DURATION );
    printf( "\n" );

    // 7. Discover existing devices
    printf( "[7] Discovering existing devices in the network...\n" );
    uint16_t sirenIn = 0x0502;
    ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 1, &sirenIn, 0, NULL );
    uint16_t btnIn = 0x0500;
    ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 1, &btnIn, 0, NULL );
    uint16_t btnOut = 0x0006;
    ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 0, NULL, 1, &btnOut );
    uint16_t occIn = 0x0406;
    ZNP_ZdoMatchDescReq( 0xFFFD, 0x0104, 1, &occIn, 0, NULL );
    printf( "\n" );

    // Start the use-case thread and one worker thread per sensor. From here on
    // the main loop only routes frames; all sensor device I/O and the siren
    // policy run on their own threads.
    UseCase_Start();
#if ENABLE_SIREN
    Siren_Start();
#endif
#if ENABLE_AQARA_BUTTON
    AqaraButton_Start();
#endif
#if ENABLE_ONICS_BUTTON
    OnicsButton_Start();
#endif
#if ENABLE_AQARA_OCCUPANCY
    AqaraOccupancy_Start();
#endif
#if ENABLE_CONTACT_SENSOR
    ContactSensor_Start();
#endif
#if ENABLE_VIBRATION_SENSOR
    VibrationSensor_Start();
#endif

    // Start CLI thread
    Cli_Start();

    printf( "✅ Coordinator is active. Permit join is OPEN (%ds).\n", PERMIT_JOIN_DURATION );
    printf( "   Enter CLI commands (type 'help' for info). Press Ctrl+C to exit.\n\n" );

    double lastRefresh = ZNP_GetCurrentTime();
    MT_FRAME_T eventFrame;

    while ( 1 )
    {
        // Pop events from the event queue (10 ms wait max)
        if ( EventQueue_Pop( &g_eventQueue, &eventFrame, 10 ) )
        {
            Main_HandleIncomingFrame( &eventFrame );
        }

        // Auto-refresh permit join if needed
        double now = ZNP_GetCurrentTime();
        if ( now - lastRefresh > PERMIT_JOIN_REFRESH )
        {
            printf( "\n[Auto-Refresh] Re-opening permit join...\n" );
            ZNP_PermitJoin( PERMIT_JOIN_DURATION );
            lastRefresh = now;
        }

#if ENABLE_VIBRATION_SENSOR
        VibrationSensor_PollAll();
#endif

        usleep( 5000 ); // Small yield
    }

    ZNP_Close();
    return 0;
}

// ---------------------------------------------------------------------------
// Incoming Frame Dispatcher
// ---------------------------------------------------------------------------
///
/// @brief  Classify and route one asynchronous ZNP frame.
///
/// Handles device announces (triggering discovery), ZDO responses (IEEE /
/// active-EP / simple-desc / match-desc, driving device classification), and AF
/// incoming messages (routed to the owning sensor's worker thread). Does no
/// sensor-specific parsing itself.
///
/// @param  frame_  The AREQ frame popped from ::g_eventQueue.
/// @return None.
///
static void Main_HandleIncomingFrame( const MT_FRAME_T *frame_ )
{
    // 1. ZDO Device Announce (cmd0: 0x45, cmd1: 0xC1)
    if ( frame_->cmd0 == 0x45 && frame_->cmd1 == 0xC1 )
    {
        if ( frame_->len < 12 )
        {
            return;
        }
        uint16_t nwkAddr = frame_->payload[2] | ( frame_->payload[3] << 8 );
        uint8_t ieeeBytes[8];
        memcpy( ieeeBytes, &frame_->payload[4], 8 );

        printf( "\n✨ [JOIN] New device joined: short=0x%04X, IEEE=", nwkAddr );
        for ( int i = 7; i >= 0; i-- )
        {
            printf( "%02x", ieeeBytes[i] );
        }
        printf( "\n" );

        Device_AddDiscoveredIeee( nwkAddr, ieeeBytes );

        // Throttle the discovery burst: a device that re-announces repeatedly
        // (e.g. a flaky/flooding siren) must NOT re-run this blocking sequence
        // every time, or it stalls the dispatcher loop and the logs "hang".
        if ( Device_ShouldQuery( nwkAddr ) )
        {


            // Trigger discovery and IEEE address resolution. The real responses
            // arrive asynchronously as events; we only need the SRSP ack here.
            uint8_t reqPay[4] = { nwkAddr & 0xFF, ( nwkAddr >> 8 ) & 0xFF, 0x01, 0x00 };
            ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 ); // IEEE_addr_req
            ZNP_ZdoActiveEpReq( nwkAddr );
            ZNP_QuerySimpleDesc( nwkAddr, 43 ); // Develco siren ep-43 fallback
        }
    }
    // 1.5 ZDO State Change Indication (cmd0: 0x45, cmd1: 0xC0)
    else if ( frame_->cmd0 == 0x45 && frame_->cmd1 == 0xC0 )
    {
        if ( frame_->len >= 1 )
        {
            uint8_t currentState = frame_->payload[0];
            printf( "\n🔄 [STATE] Coordinator state change: %d\n", currentState );
        }
    }
    // 2. ZDO Response/Callback Parser (0x45 0xFF, 0x45 0x81, 0x45 0x86)
    else if ( frame_->cmd0 == 0x45 &&
              ( frame_->cmd1 == 0xFF || frame_->cmd1 == 0x81 || frame_->cmd1 == 0x86 ) )
    {
        uint8_t status = 0;
        uint16_t shortAddr = 0;
        uint16_t clusterId = 0;
        uint16_t srcAddr = 0;

        int matchCount = 0;
        uint8_t matchList[32];

        const uint8_t *asdu = NULL;
        int asduLen = 0;

        if ( frame_->cmd1 == 0xFF )
        {
            if ( frame_->len >= 9 )
            {
                srcAddr = frame_->payload[0] | ( frame_->payload[1] << 8 );
                clusterId = frame_->payload[3] | ( frame_->payload[4] << 8 );
                asdu = &frame_->payload[9];
                asduLen = frame_->len - 9;

                if ( clusterId == 0x8001 ) // IEEE_addr_rsp
                {
                    if ( asduLen >= 11 )
                    {
                        status = asdu[0];
                        if ( status == 0 )
                        {
                            shortAddr = asdu[9] | ( asdu[10] << 8 );
                            Device_AddDiscoveredIeee( shortAddr, &asdu[1] );
                        }
                    }
                }
                else if ( clusterId == 0x8004 ) // Simple_desc_rsp
                {
                    if ( asduLen >= 4 )
                    {
                        status = asdu[0];
                        shortAddr = asdu[1] | ( asdu[2] << 8 );
                    }
                }
                else if ( clusterId == 0x8005 ) // Active_EP_rsp
                {
                    if ( asduLen >= 4 )
                    {
                        status = asdu[0];
                        shortAddr = asdu[1] | ( asdu[2] << 8 );
                        if ( status == 0 )
                        {
                            matchCount = asdu[3];
                            if ( matchCount > 32 )
                            {
                                matchCount = 32;
                            }
                            memcpy( matchList, &asdu[4], matchCount );
                        }
                    }
                }
                else if ( clusterId == 0x8006 ) // Match_Desc_rsp
                {
                    if ( asduLen >= 4 )
                    {
                        status = asdu[0];
                        shortAddr = asdu[1] | ( asdu[2] << 8 );
                        if ( status == 0 )
                        {
                            matchCount = asdu[3];
                            if ( matchCount > 32 )
                            {
                                matchCount = 32;
                            }
                            memcpy( matchList, &asdu[4], matchCount );
                        }
                    }
                }
            }
        }
        else if ( frame_->cmd1 == 0x81 ) // Direct IEEE_addr_rsp indication
        {
            clusterId = 0x8001;
            if ( frame_->len >= 11 )
            {
                status = frame_->payload[0];
                if ( status == 0 )
                {
                    shortAddr = frame_->payload[9] | ( frame_->payload[10] << 8 );
                    Device_AddDiscoveredIeee( shortAddr, &frame_->payload[1] );
                }
            }
        }
        else if ( frame_->cmd1 == 0x86 ) // Direct Match_desc_rsp indication
        {
            clusterId = 0x8006;
            if ( frame_->len >= 4 )
            {
                status = frame_->payload[0];
                shortAddr = frame_->payload[1] | ( frame_->payload[2] << 8 );
                if ( status == 0 )
                {
                    matchCount = frame_->payload[3];
                    if ( matchCount > 32 )
                    {
                        matchCount = 32;
                    }
                    memcpy( matchList, &frame_->payload[4], matchCount );
                }
            }
        }

        // Join trigger via MSG_CB (cluster 0x0013)
        if ( clusterId == 0x0013 )
        {
            printf( "\n✨ [JOIN via MSG_CB] Device announced: short=0x%04X\n", srcAddr );
            if ( asdu != NULL && asduLen >= 10 )
            {
                uint16_t annceShort = asdu[0] | ( asdu[1] << 8 );
                Device_AddDiscoveredIeee( annceShort, &asdu[2] );
            }

            // Throttle the discovery burst so a repeatedly re-announcing device
            // cannot stall the dispatcher loop (see the 0xC1 handler above).
            if ( Device_ShouldQuery( srcAddr ) )
            {

                uint8_t reqPay[4] = { srcAddr & 0xFF, ( srcAddr >> 8 ) & 0xFF, 0x01, 0x00 };
                ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 );
                ZNP_ZdoActiveEpReq( srcAddr );
                ZNP_QuerySimpleDesc( srcAddr, 43 );
            }
        }
        else if ( clusterId == 0x8001 && status == 0 )
        {
            uint8_t ieee[8];
            if ( Device_GetDiscoveredIeee( shortAddr, ieee ) )
            {
                printf( " ZDO IEEE Rsp: short=0x%04X -> IEEE=", shortAddr );
                for ( int i = 7; i >= 0; i-- )
                {
                    printf( "%02x", ieee[i] );
                }
                printf( "\n" );

                // Update modules with the resolved IEEE address
#if ENABLE_SIREN
                Siren_UpdateIeee( shortAddr, ieee );
#endif
#if ENABLE_AQARA_BUTTON
                AqaraButton_UpdateIeee( shortAddr, ieee );
#endif
#if ENABLE_ONICS_BUTTON
                OnicsButton_UpdateIeee( shortAddr, ieee );
#endif
#if ENABLE_AQARA_OCCUPANCY
                AqaraOccupancy_UpdateIeee( shortAddr, ieee );
#endif
#if ENABLE_CONTACT_SENSOR
                ContactSensor_UpdateIeee( shortAddr, ieee );
#endif
            }
        }
        else if ( clusterId == 0x8005 && status == 0 )
        {
            printf( " ZDO Active EPs Rsp: short=0x%04X, EPs=[", shortAddr );
            for ( int i = 0; i < matchCount; i++ )
            {
                printf( "%d%s", matchList[i], ( i == matchCount - 1 ) ? "" : ", " );
            }
            printf( "]\n" );
            for ( int i = 0; i < matchCount; i++ )
            {
                ZNP_QuerySimpleDesc( shortAddr, matchList[i] );
            }
        }
        else if ( clusterId == 0x8004 && asdu != NULL && asduLen >= 11 )
        {
            status = asdu[0];
            shortAddr = asdu[1] | ( asdu[2] << 8 );
            uint8_t epLen = asdu[3];
            if ( status == 0 && asduLen >= 4 + epLen )
            {
                uint8_t ep = asdu[4];
                uint16_t profileId = asdu[5] | ( asdu[6] << 8 );
                uint16_t deviceId = asdu[7] | ( asdu[8] << 8 );
                uint8_t numIn = asdu[10];

                int offset = 11;
                uint16_t inCls[32];
                int numInCls = 0;
                for ( int i = 0; i < numIn; i++ )
                {
                    if ( offset + 2 <= asduLen )
                    {
                        inCls[numInCls++] = asdu[offset] | ( asdu[offset + 1] << 8 );
                    }
                    offset += 2;
                }

                int numOutCls = 0;
                uint16_t outCls[32];
                if ( offset < asduLen )
                {
                    uint8_t numOut = asdu[offset];
                    offset += 1;
                    for ( int i = 0; i < numOut; i++ )
                    {
                        if ( offset + 2 <= asduLen )
                        {
                            outCls[numOutCls++] = asdu[offset] | ( asdu[offset + 1] << 8 );
                        }
                        offset += 2;
                    }
                }

                printf( " 📋 Device 0x%04X ep 0x%02X Profile=0x%04X DevID=0x%04X InClusters=[",
                        shortAddr, ep, profileId, deviceId );
                for ( int i = 0; i < numInCls; i++ )
                {
                    printf( "0x%04X%s", inCls[i], ( i == numInCls - 1 ) ? "" : ", " );
                }
                printf( "] OutClusters=[" );
                for ( int i = 0; i < numOutCls; i++ )
                {
                    printf( "0x%04X%s", outCls[i], ( i == numOutCls - 1 ) ? "" : ", " );
                }
                printf( "]\n" );

                if ( profileId == 0x0104 )
                {
                    bool isSiren = false;
                    for ( int i = 0; i < numInCls; i++ )
                    {
                        if ( inCls[i] == 0x0502 )
                        {
                            isSiren = true;
                            break;
                        }
                    }

                    if ( isSiren )
                    {
#if ENABLE_SIREN
                        Siren_Discover( shortAddr, ep );
#endif
                    }
#if ENABLE_SIREN
                    else if ( Siren_IsKnown( shortAddr ) )
                    {
                        printf( " 0x%04X already known as a siren - skipping button classification for ep 0x%02X\n",
                                shortAddr, ep );
                    }
#endif
                    else
                    {
                        bool isContact = false;
                        bool isVibration = false;

                        // Check if it's explicitly a Vibration/Glass Break Sensor by DevID
                        if ( deviceId == 0x0228 || deviceId == 0x022D || deviceId == 0x0101 )
                        {
                            isVibration = true;
                        }
                        else if ( deviceId == 0x0402 )
                        {
                            for ( int i = 0; i < numInCls; i++ )
                            {
                                if ( inCls[i] == 0xFC04 || inCls[i] == 0x0101 )
                                {
                                    isVibration = true;
                                    break;
                                }
                            }
                            if ( !isVibration )
                            {
                                isContact = true;
                            }
                        }

                        bool isOnics = false;
                        if ( !isContact && !isVibration )
                        {
                            for ( int i = 0; i < numInCls; i++ )
                            {
                                if ( inCls[i] == 0x000F || inCls[i] == 0x0012 )
                                {
                                    isOnics = true;
                                    break;
                                }
                            }
                        }

                        bool isAqara = false;
                        for ( int i = 0; i < numOutCls; i++ )
                        {
                            if ( outCls[i] == 0x0006 )
                            {
                                isAqara = true;
                                break;
                            }
                        }
                        for ( int i = 0; i < numInCls; i++ )
                        {
                            if ( inCls[i] == 0x0006 || inCls[i] == 0x0012 )
                            {
                                isAqara = true;
                                break;
                            }
                        }

                        bool isOccupancy = false;
                        if ( deviceId == 0x0107 )
                        {
                            isOccupancy = true;
                        }
                        for ( int i = 0; i < numInCls; i++ )
                        {
                            if ( inCls[i] == 0x0406 )
                            {
                                isOccupancy = true;
                                break;
                            }
                        }

                        if ( isOccupancy )
                        {
#if ENABLE_AQARA_OCCUPANCY
                            AqaraOccupancy_Discover( shortAddr, ep );
#endif
                        }
                        else if ( isContact )
                        {
#if ENABLE_CONTACT_SENSOR
                            ContactSensor_Discover( shortAddr, ep );
#endif
                        }
                        else if ( isVibration )
                        {
#if ENABLE_VIBRATION_SENSOR
                            VibrationSensor_Discover( shortAddr, ep );
#endif
                        }
                        else if ( isOnics )
                        {
#if ENABLE_ONICS_BUTTON
                            OnicsButton_Discover( shortAddr, ep );
#endif
                        }
                        else if ( isAqara 
#if ENABLE_ONICS_BUTTON
                                  && !OnicsButton_IsKnown( shortAddr ) 
#endif
                                  )
                        {
#if ENABLE_AQARA_BUTTON
                            AqaraButton_Discover( shortAddr, ep );
#endif
                        }
                    }
                }
            }
        }
        else if ( clusterId == 0x8006 && status == 0 )
        {
            printf( " ZDO Match Desc Rsp: short=0x%04X, endpoints=[", shortAddr );
            for ( int i = 0; i < matchCount; i++ )
            {
                printf( "%d%s", matchList[i], ( i == matchCount - 1 ) ? "" : ", " );
            }
            printf( "]\n" );
            for ( int i = 0; i < matchCount; i++ )
            {
                ZNP_QuerySimpleDesc( shortAddr, matchList[i] );
            }
        }
    }
    // 3. AF Incoming Message
    else if ( frame_->cmd0 == 0x44 && frame_->cmd1 == 0x81 )
    {
        if ( frame_->len < 17 )
        {
            return;
        }

        AF_MSG_T af;
        af.clusterId = frame_->payload[2] | ( frame_->payload[3] << 8 );
        af.srcAddr = frame_->payload[4] | ( frame_->payload[5] << 8 );
        af.srcEp = frame_->payload[6];
        af.transSeq = frame_->payload[15];
        af.dataLen = frame_->payload[16];
        memcpy( af.data, &frame_->payload[17], af.dataLen );

        if ( af.srcAddr == 0x0000 ) return;

        if ( af.clusterId != 0xFCC0 && af.clusterId != 0x0400 && af.clusterId != 0x0406 && af.clusterId != 0x0500 )
        {
            printf( "\n📩 [MSG] Incoming AF Msg: src=0x%04X ep=0x%02X cluster=0x%04X len=%d\n",
                    af.srcAddr, af.srcEp, af.clusterId, af.dataLen );
        }

        bool isKnown = false;
#if ENABLE_SIREN
        if ( Siren_IsKnown( af.srcAddr ) ) { isKnown = true; Siren_UpdateSeen( af.srcAddr ); }
#endif
#if ENABLE_AQARA_BUTTON
        if ( AqaraButton_IsKnown( af.srcAddr ) ) { isKnown = true; AqaraButton_UpdateSeen( af.srcAddr ); }
#endif
#if ENABLE_ONICS_BUTTON
        if ( OnicsButton_IsKnown( af.srcAddr ) ) { isKnown = true; OnicsButton_UpdateSeen( af.srcAddr ); }
#endif
#if ENABLE_AQARA_OCCUPANCY
        if ( AqaraOccupancy_IsKnown( af.srcAddr ) ) { isKnown = true; AqaraOccupancy_UpdateSeen( af.srcAddr ); }
#endif
#if ENABLE_CONTACT_SENSOR
        if ( ContactSensor_IsKnown( af.srcAddr ) ) { isKnown = true; ContactSensor_UpdateSeen( af.srcAddr ); }
#endif
#if ENABLE_VIBRATION_SENSOR
        if ( VibrationSensor_IsKnown( af.srcAddr ) ) { isKnown = true; VibrationSensor_UpdateSeen( af.srcAddr ); }
#endif

        if ( !isKnown && Device_ShouldQuery( af.srcAddr ) )
        {
            if ( af.srcEp == 43 )
            {
#if ENABLE_SIREN
                Siren_Discover( af.srcAddr, af.srcEp );
#endif
            }
            else
            {
                printf(" ❓ Unknown device 0x%04X sent AF message on cluster 0x%04X. Requesting Active EPs...\n", af.srcAddr, af.clusterId);
                ZNP_ZdoActiveEpReq( af.srcAddr );
            }
        }

#if ENABLE_SIREN
        if ( Siren_IsKnown( af.srcAddr ) ) { Siren_PostAf( af.srcAddr, &af ); }
#endif
#if ENABLE_AQARA_BUTTON
        else if ( AqaraButton_IsKnown( af.srcAddr ) ) { AqaraButton_PostAf( af.srcAddr, &af ); }
#endif
#if ENABLE_CONTACT_SENSOR
        else if ( ContactSensor_IsKnown( af.srcAddr ) ) { ContactSensor_PostAf( af.srcAddr, &af ); }
#endif
#if ENABLE_VIBRATION_SENSOR
        else if ( VibrationSensor_IsKnown( af.srcAddr ) ) { VibrationSensor_PostAf( af.srcAddr, &af ); }
#endif
#if ENABLE_ONICS_BUTTON
        else if ( OnicsButton_IsKnown( af.srcAddr ) ) { OnicsButton_PostAf( af.srcAddr, &af ); }
#endif
#if ENABLE_AQARA_OCCUPANCY
        else if ( AqaraOccupancy_IsKnown( af.srcAddr ) ) { AqaraOccupancy_PostAf( af.srcAddr, &af ); }
#endif
    }
}

