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
#include <unistd.h>

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
        else if ( msg->kind == SENSOR_MSG_BEEP )
        {
            // Beep sequences contain sleeps; running them here keeps the
            // use-case thread free to react to the next event immediately.
            Siren_Beep( (int)msg->shortAddr );
        }
        else if ( msg->kind == SENSOR_MSG_AF )
        {
            // The siren only sends tamper/zone traffic; a Zone Enroll Request
            // (IAS Zone 0x0500, cmd 0x01) still needs a response. Everything
            // else is informational and drives no logic.
            const AF_MSG_T *af = &msg->af;
            if ( af->dataLen >= 3 )
            {
                uint8_t fc = af->data[0];
                int hdrLen = ( fc & 0x04 ) ? 5 : 3;
                if ( af->dataLen >= hdrLen )
                {
                    uint8_t cmdId = af->data[hdrLen - 1];
                    const uint8_t *zcl = &af->data[hdrLen];
                    int zclLen = af->dataLen - hdrLen;

                    if ( af->clusterId == 0x0500 )
                    {
                        if ( cmdId == 0x01 ) // Zone Enroll Request
                        {
                            if ( zclLen >= 2 )
                            {
                                uint16_t zoneType = zcl[0] | ( zcl[1] << 8 );
                                uint8_t transSeq = af->data[hdrLen - 2];
                                Siren_HandleEnroll( af->srcAddr, af->srcEp, transSeq, zoneType );
                            }
                        }
                        else if ( cmdId == 0x00 ) // Zone Status Change Notification
                        {
                            if ( zclLen >= 2 )
                            {
                                uint16_t zoneStatus = zcl[0] | ( zcl[1] << 8 );
                                uint8_t zoneId = ( zclLen >= 4 ) ? zcl[3] : 0;
                                LOG_DEBUG("-> Zone Status Change from Siren 0x%04X: zone_status=0x%04X, zone_id=%u\n",
                                        af->srcAddr, zoneStatus, zoneId );

                                // Send Default Response
                                ZNP_SendDefaultResponse( af->srcAddr, af->srcEp, 0x0500, af->data[hdrLen - 2], 0x00, 0x00 );

                                // Bit 2 is Tamper
                                if ( zoneStatus & 0x0004 )
                                {
                                    UseCase_Post( UC_TAMPER_DETECTED, af->srcAddr, zoneStatus, 0 );
                                }
                                else
                                {
                                    UseCase_Post( UC_TAMPER_CLEARED, af->srcAddr, zoneStatus, 0 );
                                }
                            }
                        }
                    }
                    else if ( af->clusterId == 0x0001 ) // Power Configuration
                    {
                        if ( cmdId == 0x01 ) // Read Attributes Response
                        {
                            if ( zclLen >= 5 && zcl[0] == 0x20 && zcl[1] == 0x00 && zcl[2] == 0x00 )
                            {
                                uint8_t bat = zcl[4]; // Unit is 100 mV
                                LOG_DEBUG("Siren 0x%04X Battery Voltage: %.1f V\n", af->srcAddr, (float)bat / 10.0);
                            }
                        }
                    }
                }
            }
        }
        free( msg );
    }
    return NULL;
}

static uint8_t s_sirenSeq = 0;    // transaction sequence number (atomic: multiple threads send)

static uint8_t Siren_NextSeq( void )
{
    return __atomic_add_fetch( &s_sirenSeq, 1, __ATOMIC_RELAXED );
}


void Siren_Init( void )
{
    memset( g_sirens, 0, sizeof( g_sirens ) );
    g_numSirens = 0;
    MsgQueue_Init( &s_sirenInbox );
    
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

void Siren_PostBeep( int count_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_BEEP;
    msg->shortAddr = (uint16_t)count_;
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
            LOG_DEBUG("Siren discovered: short=0x%04X, ep=0x%02X\n", shortAddr_, endpoint_ );
            LOG_EVENT("SIREN", shortAddr_, "Network Join\n");
            g_sirens[g_numSirens].shortAddr = shortAddr_;
            g_sirens[g_numSirens].endpoint = endpoint_;
            g_sirens[g_numSirens].lastSeen = ZNP_GetCurrentTime();
            g_sirens[g_numSirens].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_sirens[g_numSirens].ieee );
            g_sirens[g_numSirens].configured = false;
            g_sirens[g_numSirens].volume = 2;
            g_sirens[g_numSirens].mode = 1;
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
        LOG_DEBUG( "Siren 0x%04X missing IEEE - requesting...\n", shortAddr_ );
        uint8_t reqPay[4] = { shortAddr_ & 0xFF, ( shortAddr_ >> 8 ) & 0xFF, 0x01, 0x00 };
        ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 );
        return;
    }

    LOG_DEBUG( "Configuring siren 0x%04X...\n", shortAddr_ );
    // Write coordinator's IEEE to the siren's IAS_CIE_Address attribute (0x0010).
    ZNP_WriteCieAddress( shortAddr_, endpoint, 0x14 );
    LOG_DEBUG( "Configuration sent to siren 0x%04X!\n", shortAddr_ );
}

void Siren_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ )
{
    LOG_DEBUG("-> Zone Enroll Request from Siren 0x%04X, zone_type=0x%04X\n", shortAddr_, zoneType_ );
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

void Siren_SetVolume( uint16_t shortAddr_, uint8_t volume_ )
{
    if (volume_ > 3) volume_ = 3;
    pthread_mutex_lock(&g_deviceMutex);
    bool found = false;
    for (int i = 0; i < g_numSirens; i++) {
        if (g_sirens[i].shortAddr == shortAddr_) {
            g_sirens[i].volume = volume_;
            found = true;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        printf("SUCCESS: Siren 0x%04X volume set to %d (0=low, 1=medium, 2=high, 3=very high).\n", shortAddr_, volume_);
    } else {
        printf("ERROR: Siren 0x%04X not found.\n", shortAddr_);
    }
}

uint8_t Siren_GetVolume( uint16_t shortAddr_ )
{
    uint8_t vol = 2; // default
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numSirens; i++) {
        if (g_sirens[i].shortAddr == shortAddr_) {
            vol = g_sirens[i].volume;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    return vol;
}

void Siren_SetMode( uint16_t shortAddr_, uint8_t mode_ )
{
    if (mode_ < 1 || mode_ > 6) mode_ = 1;
    pthread_mutex_lock(&g_deviceMutex);
    bool found = false;
    for (int i = 0; i < g_numSirens; i++) {
        if (g_sirens[i].shortAddr == shortAddr_) {
            g_sirens[i].mode = mode_;
            found = true;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    if (found) {
        Device_Save();
        printf("SUCCESS: Siren 0x%04X mode set to %d.\n", shortAddr_, mode_);
    } else {
        printf("ERROR: Siren 0x%04X not found.\n", shortAddr_);
    }
}

uint8_t Siren_GetMode( uint16_t shortAddr_ )
{
    uint8_t mode = 1; // default
    pthread_mutex_lock(&g_deviceMutex);
    for (int i = 0; i < g_numSirens; i++) {
        if (g_sirens[i].shortAddr == shortAddr_) {
            mode = g_sirens[i].mode;
            break;
        }
    }
    pthread_mutex_unlock(&g_deviceMutex);
    return mode;
}

void Siren_ControlAll( uint8_t warnMode_ )
{
    Siren_ControlAllDuration( warnMode_, 240 ); // default to 240 seconds
}

void Siren_Control( uint16_t shortAddr_, uint8_t warnMode_ )
{
    g_sirenActive = ( warnMode_ != 0 );
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if (g_sirens[i].shortAddr == shortAddr_) {
            uint8_t mode = (warnMode_ != 0) ? g_sirens[i].mode : 0;
            uint8_t ep = g_sirens[i].endpoint;
            uint8_t vol = g_sirens[i].volume;
            pthread_mutex_unlock( &g_deviceMutex );
            ZNP_SendSirenWarning( shortAddr_, ep, Siren_NextSeq(), mode, vol, 240 );
            return;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

void Siren_ControlAllDuration( uint8_t warnMode_, uint16_t durationSeconds_ )
{
    g_sirenActive = ( warnMode_ != 0 );

    pthread_mutex_lock( &g_deviceMutex );
    if ( g_numSirens == 0 )
    {
        if (warnMode_ != 0) {
            printf("ERROR: Failed to trigger siren. No sirens registered in the network.\n");
        }
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
        uint8_t mode = (warnMode_ != 0) ? tempSirens[i].mode : 0;
        if (warnMode_ != 0) {
            printf("SUCCESS: Siren 0x%04X triggered ON (Mode %d, Vol %d).\n", tempSirens[i].shortAddr, mode, tempSirens[i].volume);
        } else {
            printf("SUCCESS: Siren 0x%04X turned OFF.\n", tempSirens[i].shortAddr);
        }
        ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), mode, tempSirens[i].volume, durationSeconds_ );
    }
}

void Siren_TriggerAll( uint8_t mode_, uint8_t volume_, uint16_t durationSeconds_ )
{
    g_sirenActive = ( mode_ != 0 );

    pthread_mutex_lock( &g_deviceMutex );
    if ( g_numSirens == 0 )
    {
        if (mode_ != 0) {
            printf("ERROR: Failed to trigger siren. No sirens registered in the network.\n");
        }
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }

    int tempNum = g_numSirens;
    SIREN_T tempSirens[MAX_SIRENS];
    memcpy( tempSirens, g_sirens, sizeof( SIREN_T ) * g_numSirens );
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        if (mode_ != 0) {
            printf("SUCCESS: Siren 0x%04X triggered ON (Mode %d, Vol %d).\n", tempSirens[i].shortAddr, mode_, volume_);
        } else {
            printf("SUCCESS: Siren 0x%04X turned OFF.\n", tempSirens[i].shortAddr);
        }
        ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), mode_, volume_, durationSeconds_ );
    }
}

void Siren_ControlSquawk( uint8_t squawkMode_, uint8_t squawkLevel_ )
{
    (void)squawkMode_; // Unused for emulation

    pthread_mutex_lock( &g_deviceMutex );
    if ( g_numSirens == 0 )
    {
                pthread_mutex_unlock( &g_deviceMutex );
        return;
    }

    int tempNum = g_numSirens;
    SIREN_T tempSirens[MAX_SIRENS];
    memcpy( tempSirens, g_sirens, sizeof( SIREN_T ) * g_numSirens );
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        //  HARDWARE FIRMWARE BUG 
        // Even when formatted byte-for-byte perfectly according to the ZCL spec and Develco docs
        // (endpoint 1, strobe 0, mode 1, level 0), newer Frient SIRZB-110 firmwares completely 
        // ignore the native Squawk command (0x01) unless armed by a separate security panel.
        // The industry-standard workaround (used by Z2M/Home Assistant) is to emulate the chirp 
        // using the highly reliable Start Warning (0x00) command for a 1-second duration.
        ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), tempSirens[i].mode, squawkLevel_, 1 );
    }
}

void Siren_Beep( int count_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    if ( g_numSirens == 0 )
    {
        printf("ERROR: Failed to trigger siren beep. No sirens registered in the network.\n");
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    int tempNum = g_numSirens;
    SIREN_T tempSirens[MAX_SIRENS];
    memcpy( tempSirens, g_sirens, sizeof( SIREN_T ) * g_numSirens );
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        printf("SUCCESS: Siren 0x%04X emitted BEEP (x%d).\n", tempSirens[i].shortAddr, count_);
    }

    for ( int c = 0; c < count_; c++ )
    {
        for ( int i = 0; i < tempNum; i++ )
        {
            // Start warning (1 sec duration to turn it on immediately)
            ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), 1, 0, 1 ); // 1 = Burglar Mode
        }
        usleep( 100000 ); // 100ms ON time (short beep)

        for ( int i = 0; i < tempNum; i++ )
        {
            // Stop warning (mode = 0)
            ZNP_SendSirenWarning( tempSirens[i].shortAddr, tempSirens[i].endpoint, Siren_NextSeq(), 0, 0, 0 );
        }
        
        if ( c < count_ - 1 )
        {
            usleep( 300000 ); // 300ms OFF time between beeps (gap)
        }
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

uint8_t Siren_GetEndpoint( uint16_t shortAddr_ )
{
    uint8_t ep = 0;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numSirens; i++ )
    {
        if ( g_sirens[i].shortAddr == shortAddr_ )
        {
            ep = g_sirens[i].endpoint;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return ep;
}

void Siren_ReadEnvironment( uint16_t shortAddr_ )
{
    uint8_t ep = Siren_GetEndpoint(shortAddr_);
    if (ep == 0) {
        LOG_ERROR("Error: Siren 0x%04X is not registered. Cannot read environment.\n", shortAddr_);
        return;
    }

    LOG_DEBUG("Requesting Environment Data (Battery) from Siren 0x%04X on EP 0x%02X...\n", shortAddr_, ep);
  
    // Read Battery Voltage (Cluster 0x0001, Attr 0x0020)
    uint8_t zclFrameBat[5];
    zclFrameBat[0] = 0x00; 
    zclFrameBat[1] = 0xD1; // Trans seq
    zclFrameBat[2] = 0x00; // Read Attributes
    zclFrameBat[3] = 0x20; // Attr 0x0020
    zclFrameBat[4] = 0x00; 
    ZNP_AfDataRequestExt(2, shortAddr_, ep, 0, 8, 0x0001, 0xD1, 0, 30, zclFrameBat, 5);
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
