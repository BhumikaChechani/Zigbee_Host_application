#include "config.h"
#if ENABLE_SIREN
///
/// @file   siren.c
/// @brief  Implementation of the Smart Siren module (see siren.h).
///
#include "siren.h"
#include "msg_queue.h"
#include "usecase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

SIREN_T g_sirens[MAX_SIRENS];
int g_numSirens = 0;

// Per-sensor worker thread + inbox. All siren device I/O (setup, tamper enroll)
// runs here so it never blocks the dispatcher or the other sensors.
static MSG_QUEUE_T s_sirenInbox;
static pthread_t s_sirenThread;

///
/// @brief  Siren worker thread: run setup on ASSIGN and answer tamper enrolls.
/// @param  arg_  Unused thread argument.
/// @return NULL (runs until process exit).
///
static void *Siren_Thread( void *arg_ )
{
    (void)arg_;
    while ( 1 )
    {
        SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop( &s_sirenInbox );
        if ( msg == NULL )
        {
            continue;
        }
        if ( msg->kind == SENSOR_MSG_ASSIGN )
        {
            Siren_Setup( msg->shortAddr );
        }
        else if ( msg->kind == SENSOR_MSG_AF )
        {
            // The siren only sends tamper/zone traffic; a Zone Enroll Request
            // (IAS Zone 0x0500, cmd 0x01) still needs a response. Everything
            // else is informational and drives no logic.
            const AF_MSG_T *af = &msg->af;
            if ( af->clusterId == 0x0500 && af->dataLen >= 3 )
            {
                uint8_t fc = af->data[0];
                int hdrLen = ( fc & 0x04 ) ? 5 : 3;
                if ( af->dataLen >= hdrLen && af->data[hdrLen - 1] == 0x01 )
                {
                    const uint8_t *zcl = &af->data[hdrLen];
                    int zclLen = af->dataLen - hdrLen;
                    if ( zclLen >= 2 )
                    {
                        uint16_t zoneType = zcl[0] | ( zcl[1] << 8 );
                        uint8_t transSeq = af->data[hdrLen - 2];
                        Siren_HandleEnroll( af->srcAddr, af->srcEp, transSeq, zoneType );
                    }
                }
            }
        }
        free( msg );
    }
    return NULL;
}

static uint8_t s_sirenVolume = 2; // high by default
static uint8_t s_sirenMode = 1;   // burglar by default

static const char SIREN_CONFIG_FILE[] = "siren_config.txt";

static void Siren_LoadConfig(void)
{
    FILE *f = fopen(SIREN_CONFIG_FILE, "r");
    if (f)
    {
        int v, m;
        int parsed = fscanf(f, "%d %d", &v, &m);
        if (parsed >= 1)
        {
            if (v >= 0 && v <= 3) s_sirenVolume = (uint8_t)v;
        }
        if (parsed >= 2)
        {
            if (m >= 1 && m <= 6) s_sirenMode = (uint8_t)m;
        }
        fclose(f);
    }
}

static void Siren_SaveConfig(void)
{
    FILE *f = fopen(SIREN_CONFIG_FILE, "w");
    if (f)
    {
        fprintf(f, "%d %d\n", s_sirenVolume, s_sirenMode);
        fclose(f);
    }
}

void Siren_Init( void )
{
    memset( g_sirens, 0, sizeof( g_sirens ) );
    g_numSirens = 0;
    MsgQueue_Init( &s_sirenInbox );
    Siren_LoadConfig();
}

void Siren_Start( void )
{
    pthread_create( &s_sirenThread, NULL, Siren_Thread, NULL );
}

void Siren_PostAssign( uint16_t shortAddr_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_ASSIGN;
    msg->shortAddr = shortAddr_;
    MsgQueue_Push( &s_sirenInbox, msg );
}

void Siren_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_AF;
    msg->shortAddr = shortAddr_;
    msg->af = *af_;
    MsgQueue_Push( &s_sirenInbox, msg );
}

// Register (or refresh) a siren in the registry. This runs in the dispatcher
// and only touches shared state; the actual device setup I/O is handed off to
// the siren worker thread via Siren_PostAssign().
void Siren_Discover( uint16_t shortAddr_, uint8_t endpoint_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
    }

    bool changed = false;
    if ( idx == -1 )
    {
        if ( g_numSirens < MAX_SIRENS )
        {
            printf( " Siren discovered: short=0x%04X, ep=0x%02X\n", shortAddr_, endpoint_ );
            g_sirens[g_numSirens].shortAddr = shortAddr_;
            g_sirens[g_numSirens].endpoint = endpoint_;
            g_sirens[g_numSirens].lastSeen = ZNP_GetCurrentTime();
            g_sirens[g_numSirens].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_sirens[g_numSirens].ieee );
            g_sirens[g_numSirens].configured = false;
            g_numSirens++;
            changed = true;
        }
    }
    else
    {
        if ( g_sirens[idx].endpoint != endpoint_ )
        {
            g_sirens[idx].endpoint = endpoint_;
            changed = true;
        }
        g_sirens[idx].lastSeen = ZNP_GetCurrentTime();
        if ( !g_sirens[idx].hasIeee )
        {
            g_sirens[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_sirens[idx].ieee );
            if ( g_sirens[idx].hasIeee )
            {
                changed = true;
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    if ( changed )
    {
        Device_Save();
    }
    // Hand off to the siren thread to run setup (resolve IEEE if needed).
    Siren_PostAssign( shortAddr_ );
}

void Siren_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ )
{
    bool found = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            memcpy( g_sirens[i].ieee, ieee_, 8 );
            g_sirens[i].hasIeee = true;
            found = true;
            break;
        }
    }
    if ( found )
    {
        // Collapse stale duplicates: the same physical device (same IEEE) that
        // rejoined earlier under a different (now dead) short address.
        for ( int i = g_numSirens - 1; i >= 0; i-- )
        {
            if ( g_sirens[i].shortAddr != shortAddr_ && g_sirens[i].hasIeee &&
                 memcmp( g_sirens[i].ieee, ieee_, 8 ) == 0 )
            {
                for ( int j = i; j < g_numSirens - 1; j++ )
                {
                    g_sirens[j] = g_sirens[j + 1];
                }
                g_numSirens--;
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    if ( found )
    {
        Device_Save();
        Siren_PostAssign( shortAddr_ ); // run setup on the siren thread
    }
}

// Runs on the siren worker thread. Resolves the IEEE (requesting it if needed)
// and, once known, writes the coordinator's CIE address to the siren so its
// tamper zone can enroll. Idempotent via the 'configured' flag.
void Siren_Setup( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
    }
    if ( idx == -1 )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    if ( g_sirens[idx].configured )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    if ( !g_sirens[idx].hasIeee )
    {
        g_sirens[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_sirens[idx].ieee );
    }
    bool hasIeee = g_sirens[idx].hasIeee;
    uint8_t endpoint = g_sirens[idx].endpoint;
    if ( hasIeee )
    {
        g_sirens[idx].configured = true;
    }
    pthread_mutex_unlock( &g_deviceMutex );

    if ( !hasIeee )
    {
        // Ask for the IEEE; the IEEE response re-triggers setup via UpdateIeee.
        printf( "Siren 0x%04X missing IEEE - requesting...\n", shortAddr_ );
        uint8_t reqPay[4] = { shortAddr_ & 0xFF, ( shortAddr_ >> 8 ) & 0xFF, 0x01, 0x00 };
        ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 );
        return;
    }

    printf( "Configuring siren 0x%04X...\n", shortAddr_ );
    // Write coordinator's IEEE to the siren's IAS_CIE_Address attribute (0x0010).
    ZNP_WriteCieAddress( shortAddr_, endpoint, 0x14 );
    printf( "Configuration sent to siren 0x%04X!\n", shortAddr_ );
}

void Siren_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ )
{
    printf( "   -> Zone Enroll Request from Siren 0x%04X, zone_type=0x%04X\n", shortAddr_, zoneType_ );
    uint8_t zoneId = g_nextZoneId++;

    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            g_sirens[i].zoneId = zoneId;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    Device_Save();

    ZNP_SendZoneEnrollResponse( shortAddr_, endpoint_, transSeq_, zoneId );
}

void Siren_SetVolume( uint8_t volume_ )
{
    if (volume_ > 3) volume_ = 3;
    s_sirenVolume = volume_;
    printf( "Siren global volume set to %d (0=low, 1=medium, 2=high, 3=very high)\n", volume_ );
    Siren_SaveConfig();
}

uint8_t Siren_GetVolume( void )
{
    return s_sirenVolume;
}

void Siren_SetMode( uint8_t mode_ )
{
    if (mode_ < 1 || mode_ > 6) mode_ = 1;
    s_sirenMode = mode_;
    printf( "Siren global mode set to %d\n", mode_ );
    Siren_SaveConfig();
}

uint8_t Siren_GetMode( void )
{
    return s_sirenMode;
}

void Siren_ControlAll( uint8_t warnMode_ )
{
    g_sirenActive = ( warnMode_ != 0 );

    pthread_mutex_lock( &g_deviceMutex );
    if ( g_numSirens == 0 )
    {
        printf( "⚠️ No sirens registered yet.\n" );
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }

    // Copy locally to avoid calling blocking functions under lock
    int tempNum = g_numSirens;
    SIREN_T tempSirens[MAX_SIRENS];
    memcpy( tempSirens, g_sirens, sizeof( SIREN_T ) * g_numSirens );
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        uint8_t mode = (warnMode_ != 0) ? s_sirenMode : 0;
        ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, 0xAA, mode, s_sirenVolume, 240 );
    }
}

void Siren_PrintStatus( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    printf( "Registered Sirens (%d):\n", g_numSirens );
    double now = ZNP_GetCurrentTime();
    for ( int i = 0; i < g_numSirens; i++ )
    {
        printf( "  - 0x%04X: ep=0x%02X, last_seen=%.1fs ago\n",
                g_sirens[i].shortAddr, g_sirens[i].endpoint, now - g_sirens[i].lastSeen );
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

bool Siren_IsKnown( uint16_t shortAddr_ )
{
    bool known = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            known = true;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return known;
}

void Siren_UpdateSeen( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            g_sirens[i].lastSeen = ZNP_GetCurrentTime();
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

void Siren_DiscoverAllActiveEp( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    int tempNum = g_numSirens;
    uint16_t tempAddrs[MAX_SIRENS];
    for ( int i = 0; i < g_numSirens; i++ )
    {
        tempAddrs[i] = g_sirens[i].shortAddr;
    }
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        ZNP_ZdoActiveEpReq( tempAddrs[i] );
        ZNP_QuerySimpleDesc( tempAddrs[i], 43 );
    }
}
#endif
