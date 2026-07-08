#include "config.h"
#if ENABLE_ONICS_BUTTON
///
/// @file   onics_button.c
/// @brief  Implementation of the Onics panic button module (see onics_button.h).
///
#include "onics_button.h"
#include "msg_queue.h"
#include "usecase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

ONICS_BUTTON_T g_onicsButtons[MAX_ONICS_BUTTONS];
int g_numOnicsButtons = 0;

// Per-sensor worker thread + inbox. Onics device I/O (bind + CIE + activation)
// and command parsing run here; presses and panic alarms are forwarded to the
// use-case layer, and Zone Enroll Requests are answered from this thread.
static MSG_QUEUE_T s_onicsInbox;
static pthread_t s_onicsThread;

///
/// @brief  Parse an Onics AF frame: On/Off press, zone enroll, or panic status.
/// @param  af_  The decoded AF message (On/Off 0x0006 or IAS Zone 0x0500).
/// @return None.
///
static void OnicsButton_HandleAf( const AF_MSG_T *af_ )
{
    if ( af_->dataLen < 3 )
    {
        return;
    }
    uint8_t fc = af_->data[0];
    int hdrLen = ( fc & 0x04 ) ? 5 : 3;
    if ( af_->dataLen < hdrLen )
    {
        return;
    }
    uint8_t cmdId = af_->data[hdrLen - 1];

    if ( af_->clusterId == 0x0006 )
    {
        // Some Onics buttons emit On/Off; treat it as a press.
        printf( "👉 [ONICS BUTTON] On/Off Command received: cmd_id=0x%02X\n", cmdId );
        if ( cmdId == 0x01 )
        {
            UseCase_Post( UC_BUTTON_ON, af_->srcAddr, cmdId );
        }
        else if ( cmdId == 0x00 )
        {
            UseCase_Post( UC_BUTTON_OFF, af_->srcAddr, cmdId );
        }
        else if ( cmdId == 0x02 )
        {
            UseCase_Post( UC_BUTTON_TOGGLE, af_->srcAddr, cmdId );
        }
    }
    else if ( af_->clusterId == 0x0500 )
    {
        const uint8_t *zcl = &af_->data[hdrLen];
        int zclLen = af_->dataLen - hdrLen;
        if ( cmdId == 0x01 ) // Zone Enroll Request
        {
            if ( zclLen >= 2 )
            {
                uint16_t zoneType = zcl[0] | ( zcl[1] << 8 );
                uint8_t transSeq = af_->data[hdrLen - 2];
                OnicsButton_HandleEnroll( af_->srcAddr, af_->srcEp, transSeq, zoneType );
            }
        }
        else if ( cmdId == 0x00 ) // Zone Status Change Notification
        {
            if ( zclLen >= 4 )
            {
                uint16_t zoneStatus = zcl[0] | ( zcl[1] << 8 );
                uint8_t zoneId = zcl[3];
                OnicsButton_HandleStatus( af_->srcAddr, zoneStatus, zoneId );
            }
        }
    }
}

///
/// @brief  Onics worker thread: run setup on ASSIGN, parse traffic on AF.
/// @param  arg_  Unused thread argument.
/// @return NULL (runs until process exit).
///
static void *OnicsButton_Thread( void *arg_ )
{
    (void)arg_;
    while ( 1 )
    {
        SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop( &s_onicsInbox );
        if ( msg == NULL )
        {
            continue;
        }
        if ( msg->kind == SENSOR_MSG_ASSIGN )
        {
            OnicsButton_Setup( msg->shortAddr );
        }
        else if ( msg->kind == SENSOR_MSG_AF )
        {
            OnicsButton_HandleAf( &msg->af );
        }
        free( msg );
    }
    return NULL;
}

void OnicsButton_Init( void )
{
    memset( g_onicsButtons, 0, sizeof( g_onicsButtons ) );
    g_numOnicsButtons = 0;
    MsgQueue_Init( &s_onicsInbox );
}

void OnicsButton_Start( void )
{
    pthread_create( &s_onicsThread, NULL, OnicsButton_Thread, NULL );
}

void OnicsButton_PostAssign( uint16_t shortAddr_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_ASSIGN;
    msg->shortAddr = shortAddr_;
    MsgQueue_Push( &s_onicsInbox, msg );
}

void OnicsButton_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_AF;
    msg->shortAddr = shortAddr_;
    msg->af = *af_;
    MsgQueue_Push( &s_onicsInbox, msg );
}

void OnicsButton_Discover( uint16_t shortAddr_, uint8_t endpoint_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        if ( g_onicsButtons[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
    }

    bool changed = false;
    if ( idx == -1 )
    {
        if ( g_numOnicsButtons < MAX_ONICS_BUTTONS )
        {
            printf( " Onics Button discovered: short=0x%04X, ep=0x%02X\n", shortAddr_, endpoint_ );
            g_onicsButtons[g_numOnicsButtons].shortAddr = shortAddr_;
            g_onicsButtons[g_numOnicsButtons].endpoint = endpoint_;
            g_onicsButtons[g_numOnicsButtons].lastSeen = ZNP_GetCurrentTime();
            g_onicsButtons[g_numOnicsButtons].zoneId = -1;
            g_onicsButtons[g_numOnicsButtons].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_onicsButtons[g_numOnicsButtons].ieee );
            g_onicsButtons[g_numOnicsButtons].configured = false;
            g_numOnicsButtons++;
            changed = true;
        }
    }
    else
    {
        if ( g_onicsButtons[idx].endpoint != endpoint_ )
        {
            g_onicsButtons[idx].endpoint = endpoint_;
            changed = true;
        }
        g_onicsButtons[idx].lastSeen = ZNP_GetCurrentTime();
        if ( !g_onicsButtons[idx].hasIeee )
        {
            g_onicsButtons[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_onicsButtons[idx].ieee );
            if ( g_onicsButtons[idx].hasIeee )
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
    OnicsButton_PostAssign( shortAddr_ );
}

void OnicsButton_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ )
{
    bool found = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        if ( g_onicsButtons[i].shortAddr == shortAddr_ )
        {
            memcpy( g_onicsButtons[i].ieee, ieee_, 8 );
            g_onicsButtons[i].hasIeee = true;
            found = true;
            break;
        }
    }
    if ( found )
    {
        // Collapse stale duplicates: same physical device (same IEEE) that
        // rejoined earlier under a different (now dead) short address.
        for ( int i = g_numOnicsButtons - 1; i >= 0; i-- )
        {
            if ( g_onicsButtons[i].shortAddr != shortAddr_ && g_onicsButtons[i].hasIeee &&
                 memcmp( g_onicsButtons[i].ieee, ieee_, 8 ) == 0 )
            {
                for ( int j = i; j < g_numOnicsButtons - 1; j++ )
                {
                    g_onicsButtons[j] = g_onicsButtons[j + 1];
                }
                g_numOnicsButtons--;
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    if ( found )
    {
        Device_Save();
        OnicsButton_PostAssign( shortAddr_ ); // run setup on the onics thread
    }
}

void OnicsButton_Setup( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        if ( g_onicsButtons[i].shortAddr == shortAddr_ )
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
    if ( g_onicsButtons[idx].configured )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    if ( !g_onicsButtons[idx].hasIeee )
    {
        g_onicsButtons[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_onicsButtons[idx].ieee );
    }
    if ( !g_onicsButtons[idx].hasIeee )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        // Request IEEE; the response re-triggers setup via UpdateIeee.
        printf( "Onics button 0x%04X missing IEEE - requesting...\n", shortAddr_ );
        uint8_t reqPay[4] = { shortAddr_ & 0xFF, ( shortAddr_ >> 8 ) & 0xFF, 0x01, 0x00 };
        ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 );
        return;
    }

    uint8_t buttonIeee[8];
    uint8_t endpoint = g_onicsButtons[idx].endpoint;
    memcpy( buttonIeee, g_onicsButtons[idx].ieee, 8 );
    g_onicsButtons[idx].configured = true;
    pthread_mutex_unlock( &g_deviceMutex );

    printf( "Configuring Onics button 0x%04X...\n", shortAddr_ );

    // 1. Bind On/Off cluster output (0x0006) to coordinator endpoint 8.
    ZNP_ZdoBindReq( shortAddr_, buttonIeee, endpoint, 0x0006, g_coordinatorIeee, 8 );
    usleep( 500000 );

    // 2. Write coordinator's IEEE to button's IAS_CIE_Address attribute (0x0010).
    ZNP_WriteCieAddress( shortAddr_, endpoint, 0x12 );
    usleep( 500000 );

    // 3. Send Onics activation write (Binary Input cluster 0x000F).
    ZNP_SendButtonActivation( shortAddr_, endpoint, 0x13 );
    printf( "Configuration sent to Onics button 0x%04X!\n", shortAddr_ );
}

void OnicsButton_HandleEnroll( uint16_t shortAddr_, uint8_t endpoint_, uint8_t transSeq_, uint16_t zoneType_ )
{
    printf( "   -> Zone Enroll Request from Onics 0x%04X, zone_type=0x%04X\n", shortAddr_, zoneType_ );
    uint8_t zoneId = g_nextZoneId++;

    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        if ( g_onicsButtons[i].shortAddr == shortAddr_ )
        {
            g_onicsButtons[i].zoneId = zoneId;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    Device_Save();

    ZNP_SendZoneEnrollResponse( shortAddr_, endpoint_, transSeq_, zoneId );
}

// Parse a panic alarm and forward it to the use-case layer (which owns the
// siren policy). The button module never drives the siren directly.
void OnicsButton_HandleStatus( uint16_t shortAddr_, uint16_t zoneStatus_, uint8_t zoneId_ )
{
    printf( "   -> Zone Status Change from Onics 0x%04X: zone_status=0x%04X, zone_id=%d\n",
            shortAddr_, zoneStatus_, zoneId_ );

    bool alarm = ( zoneStatus_ & 0x0003 ) != 0;
    if ( alarm )
    {
        UseCase_Post( UC_PANIC_SET, shortAddr_, zoneStatus_ );
    }
    else
    {
        UseCase_Post( UC_PANIC_CLEAR, shortAddr_, zoneStatus_ );
    }
}

void OnicsButton_PrintStatus( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    printf( "Registered Onics Buttons (%d):\n", g_numOnicsButtons );
    double now = ZNP_GetCurrentTime();
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        printf( "  - 0x%04X: IEEE=", g_onicsButtons[i].shortAddr );
        if ( g_onicsButtons[i].hasIeee )
        {
            for ( int j = 7; j >= 0; j-- )
            {
                printf( "%02x", g_onicsButtons[i].ieee[j] );
            }
        }
        else
        {
            printf( "Unknown" );
        }
        printf( ", ep=0x%02X, zone_id=%d, last_seen=%.1fs ago\n",
                g_onicsButtons[i].endpoint, g_onicsButtons[i].zoneId, now - g_onicsButtons[i].lastSeen );
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

bool OnicsButton_IsKnown( uint16_t shortAddr_ )
{
    bool known = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        if ( g_onicsButtons[i].shortAddr == shortAddr_ )
        {
            known = true;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return known;
}

void OnicsButton_UpdateSeen( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        if ( g_onicsButtons[i].shortAddr == shortAddr_ )
        {
            g_onicsButtons[i].lastSeen = ZNP_GetCurrentTime();
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

void OnicsButton_DiscoverAllActiveEp( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    int tempNum = g_numOnicsButtons;
    uint16_t tempAddrs[MAX_ONICS_BUTTONS];
    for ( int i = 0; i < g_numOnicsButtons; i++ )
    {
        tempAddrs[i] = g_onicsButtons[i].shortAddr;
    }
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        ZNP_ZdoActiveEpReq( tempAddrs[i] );
        ZNP_QuerySimpleDesc( tempAddrs[i], 1 );
    }
}
#endif
