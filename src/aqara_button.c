#include "config.h"
#if ENABLE_AQARA_BUTTON
///
/// @file   aqara_button.c
/// @brief  Implementation of the Aqara switch module (see aqara_button.h).
///
#include "aqara_button.h"
#include "msg_queue.h"
#include "usecase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

AQARA_BUTTON_T g_aqaraButtons[MAX_AQARA_BUTTONS];
int g_numAqaraButtons = 0;

// Per-sensor worker thread + inbox. Aqara device I/O (bind setup) and press
// parsing run here; the parsed press is forwarded to the use-case layer, never
// straight to the siren.
static MSG_QUEUE_T s_aqaraInbox;
static pthread_t s_aqaraThread;

///
/// @brief  Parse an On/Off AF frame and forward it as a use-case event.
/// @param  af_  The decoded AF message (expected On/Off cluster 0x0006).
/// @return None.
///
static void AqaraButton_HandleAf( const AF_MSG_T *af_ )
{
    if ( af_->clusterId != 0x0006 || af_->dataLen < 3 )
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
    AqaraButton_HandleCommand( af_->srcAddr, cmdId );
}

///
/// @brief  Aqara worker thread: run setup on ASSIGN, parse presses on AF.
/// @param  arg_  Unused thread argument.
/// @return NULL (runs until process exit).
///
static void *AqaraButton_Thread( void *arg_ )
{
    (void)arg_;
    while ( 1 )
    {
        SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop( &s_aqaraInbox );
        if ( msg == NULL )
        {
            continue;
        }
        if ( msg->kind == SENSOR_MSG_ASSIGN )
        {
            AqaraButton_Setup( msg->shortAddr );
        }
        else if ( msg->kind == SENSOR_MSG_AF )
        {
            AqaraButton_HandleAf( &msg->af );
        }
        free( msg );
    }
    return NULL;
}

void AqaraButton_Init( void )
{
    memset( g_aqaraButtons, 0, sizeof( g_aqaraButtons ) );
    g_numAqaraButtons = 0;
    MsgQueue_Init( &s_aqaraInbox );
}

void AqaraButton_Start( void )
{
    pthread_create( &s_aqaraThread, NULL, AqaraButton_Thread, NULL );
}

void AqaraButton_PostAssign( uint16_t shortAddr_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_ASSIGN;
    msg->shortAddr = shortAddr_;
    MsgQueue_Push( &s_aqaraInbox, msg );
}

void AqaraButton_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_AF;
    msg->shortAddr = shortAddr_;
    msg->af = *af_;
    MsgQueue_Push( &s_aqaraInbox, msg );
}

// Register (or refresh) an Aqara button. Runs in the dispatcher; only touches
// shared state. Device setup I/O is handed to the aqara worker thread.
void AqaraButton_Discover( uint16_t shortAddr_, uint8_t endpoint_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        if ( g_aqaraButtons[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
    }

    bool changed = false;
    if ( idx == -1 )
    {
        if ( g_numAqaraButtons < MAX_AQARA_BUTTONS )
        {
            LOG_DEBUG("Aqara Button discovered: short=0x%04X, ep=0x%02X\n", shortAddr_, endpoint_ );
            LOG_EVENT("AQARA BTN", shortAddr_, "Network Join\n");
            g_aqaraButtons[g_numAqaraButtons].shortAddr = shortAddr_;
            g_aqaraButtons[g_numAqaraButtons].endpoint = endpoint_;
            g_aqaraButtons[g_numAqaraButtons].lastSeen = ZNP_GetCurrentTime();
            g_aqaraButtons[g_numAqaraButtons].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_aqaraButtons[g_numAqaraButtons].ieee );
            g_aqaraButtons[g_numAqaraButtons].configured = false;
            g_numAqaraButtons++;
            changed = true;
        }
    }
    else
    {
        if ( g_aqaraButtons[idx].endpoint != endpoint_ )
        {
            g_aqaraButtons[idx].endpoint = endpoint_;
            changed = true;
        }
        g_aqaraButtons[idx].lastSeen = ZNP_GetCurrentTime();
        if ( !g_aqaraButtons[idx].hasIeee )
        {
            g_aqaraButtons[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_aqaraButtons[idx].ieee );
            if ( g_aqaraButtons[idx].hasIeee )
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
    AqaraButton_PostAssign( shortAddr_ );
}

void AqaraButton_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ )
{
    bool found = false;
    pthread_mutex_lock( &g_deviceMutex );
    int targetIdx = -1;
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        if ( g_aqaraButtons[i].shortAddr == shortAddr_ )
        {
            memcpy( g_aqaraButtons[i].ieee, ieee_, 8 );
            g_aqaraButtons[i].hasIeee = true;
            targetIdx = i;
            found = true;
            break;
        }
    }
    if ( found )
    {
        // Collapse stale duplicates: same physical device (same IEEE) that
        // rejoined earlier under a different (now dead) short address. This is
        // what caused several aqara entries to share one IEEE in devices.txt.
        for ( int i = g_numAqaraButtons - 1; i >= 0; i-- )
        {
            if ( g_aqaraButtons[i].shortAddr != shortAddr_ && g_aqaraButtons[i].hasIeee &&
                 memcmp( g_aqaraButtons[i].ieee, ieee_, 8 ) == 0 )
            {
                // Preserve configuration from the old (now stale) entry
                g_aqaraButtons[targetIdx].configured = g_aqaraButtons[i].configured;

                for ( int j = i; j < g_numAqaraButtons - 1; j++ )
                {
                    g_aqaraButtons[j] = g_aqaraButtons[j + 1];
                }
                g_numAqaraButtons--;

                if (targetIdx > i) {
                    targetIdx--;
                }
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    if ( found )
    {
        Device_Save();
        AqaraButton_PostAssign( shortAddr_ ); // run setup on the aqara thread
    }
}

void AqaraButton_Setup( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        if ( g_aqaraButtons[i].shortAddr == shortAddr_ )
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
    if ( g_aqaraButtons[idx].configured )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    if ( !g_aqaraButtons[idx].hasIeee )
    {
        g_aqaraButtons[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_aqaraButtons[idx].ieee );
    }
    if ( !g_aqaraButtons[idx].hasIeee )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        // Request IEEE; the response re-triggers setup via UpdateIeee.
        LOG_DEBUG( "Aqara button 0x%04X missing IEEE - requesting...\n", shortAddr_ );
        uint8_t reqPay[4] = { shortAddr_ & 0xFF, ( shortAddr_ >> 8 ) & 0xFF, 0x01, 0x00 };
        ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 );
        return;
    }

    uint8_t buttonIeee[8];
    uint8_t endpoint = g_aqaraButtons[idx].endpoint;
    memcpy( buttonIeee, g_aqaraButtons[idx].ieee, 8 );
    g_aqaraButtons[idx].configured = true;
    pthread_mutex_unlock( &g_deviceMutex );

    LOG_DEBUG( "Configuring Aqara button 0x%04X...\n", shortAddr_ );

    // Bind On/Off cluster output (0x0006) to coordinator endpoint 8.
    ZNP_ZdoBindReq( shortAddr_, buttonIeee, endpoint, 0x0006, g_coordinatorIeee, 8 );
    LOG_DEBUG( "Configuration sent to Aqara button 0x%04X!\n", shortAddr_ );
}

// Parse a press and forward it to the use-case layer. The button module does
// NOT drive the siren itself - that policy lives in usecase.c.
void AqaraButton_HandleCommand( uint16_t shortAddr_, uint8_t cmdId_ )
{
    LOG_DEBUG("[AQARA BUTTON] Command received: cmd_id=0x%02X from src=0x%04X\n", cmdId_, shortAddr_ );
    if ( cmdId_ == 0x01 ) // On
    {
        UseCase_Post( UC_BUTTON_ON, shortAddr_, cmdId_, 0 );
    }
    else if ( cmdId_ == 0x00 ) // Off
    {
        UseCase_Post( UC_BUTTON_OFF, shortAddr_, cmdId_, 0 );
    }
    else if ( cmdId_ == 0x02 ) // Toggle
    {
        UseCase_Post( UC_BUTTON_TOGGLE, shortAddr_, cmdId_, 0 );
    }
}

void AqaraButton_PrintStatus( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    printf( "Registered Aqara Buttons (%d):\n", g_numAqaraButtons );
    double now = ZNP_GetCurrentTime();
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        printf( "  - 0x%04X: IEEE=", g_aqaraButtons[i].shortAddr );
        if ( g_aqaraButtons[i].hasIeee )
        {
            for ( int j = 7; j >= 0; j-- )
            {
                printf( "%02x", g_aqaraButtons[i].ieee[j] );
            }
        }
        else
        {
            printf( "Unknown" );
        }
        printf( ", ep=0x%02X, last_seen=%.1fs ago\n",
                g_aqaraButtons[i].endpoint, now - g_aqaraButtons[i].lastSeen );
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

bool AqaraButton_IsKnown( uint16_t shortAddr_ )
{
    bool known = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        if ( g_aqaraButtons[i].shortAddr == shortAddr_ )
        {
            known = true;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return known;
}

void AqaraButton_UpdateSeen( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        if ( g_aqaraButtons[i].shortAddr == shortAddr_ )
        {
            g_aqaraButtons[i].lastSeen = ZNP_GetCurrentTime();
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

void AqaraButton_DiscoverAllActiveEp( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    int tempNum = g_numAqaraButtons;
    uint16_t tempAddrs[MAX_AQARA_BUTTONS];
    for ( int i = 0; i < g_numAqaraButtons; i++ )
    {
        tempAddrs[i] = g_aqaraButtons[i].shortAddr;
    }
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        ZNP_ZdoActiveEpReq( tempAddrs[i] );
        ZNP_QuerySimpleDesc( tempAddrs[i], 1 );
    }
}
#endif
