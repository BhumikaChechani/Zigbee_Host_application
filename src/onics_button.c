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
        // The hardware sends BOTH a 0x0500 Zone Status and a 0x0006 Toggle on the first press.
        // It DOES NOT send another 0x0500 to clear the alarm.
        // We therefore use subsequent 0x0006 toggles as the "Manual Clear" for the panic state.
        // We must debounce this against the initial press so it doesn't instantly clear.
        double now = ZNP_GetCurrentTime();
        double timeSincePanic = 999.0;
        
        pthread_mutex_lock( &g_deviceMutex );
        for ( int i = 0; i < g_numOnicsButtons; i++ )
        {
            if ( g_onicsButtons[i].shortAddr == af_->srcAddr )
            {
                timeSincePanic = now - g_onicsButtons[i].lastPanicTime;
                break;
            }
        }
        pthread_mutex_unlock( &g_deviceMutex );

        if ( timeSincePanic > 2.0 )
        {
            printf( "👉 [ONICS BUTTON] Manual clear via On/Off toggle (cmd=0x%02X)\n", cmdId );
            // Post a raw zone status of 0x0000 (all clear)
            UseCase_Post( UC_PANIC_CLEAR, af_->srcAddr, 0x0000 );
        }
        else
        {
            printf( "👉 [ONICS BUTTON] Ignored On/Off toggle (cmd=0x%02X) - debouncing simultaneous IAS Panic alarm\n", cmdId );
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
    else if ( af_->clusterId == 0x0001 ) // Power Configuration
    {
        if ( cmdId == 0x01 ) // Read Attributes Response
        {
            const uint8_t *zcl = &af_->data[hdrLen];
            int zclLen = af_->dataLen - hdrLen;
            if ( zclLen >= 5 && zcl[0] == 0x20 && zcl[1] == 0x00 && zcl[2] == 0x00 )
            {
                uint8_t bat = zcl[4]; // Unit is 100 mV
                printf( "🔋 Onics Button 0x%04X Battery Voltage: %.1f V\n", af_->srcAddr, (float)bat / 10.0 );
            }
        }
    }
    else if ( af_->clusterId == 0x0402 ) // Temperature Measurement
    {
        if ( cmdId == 0x01 || cmdId == 0x0A )
        {
            const uint8_t *zcl = &af_->data[hdrLen];
            int zclLen = af_->dataLen - hdrLen;
            if ( cmdId == 0x01 && zclLen >= 6 && zcl[0] == 0x00 && zcl[1] == 0x00 && zcl[2] == 0x00 )
            {
                int16_t temp = (int16_t)( zcl[4] | ( zcl[5] << 8 ) );
                printf( "🌡️ Onics Button 0x%04X Temperature: %.2f °C\n", af_->srcAddr, (float)temp / 100.0 );
            }
            else if ( cmdId == 0x0A && zclLen >= 5 && zcl[0] == 0x00 && zcl[1] == 0x00 )
            {
                int16_t temp = (int16_t)( zcl[3] | ( zcl[4] << 8 ) );
                printf( "🌡️ Onics Button 0x%04X Temperature Report: %.2f °C\n", af_->srcAddr, (float)temp / 100.0 );
            }
        }
    }
    else if ( af_->clusterId == 0x000F ) // Binary Input - activation write response
    {
        // cmd 0x04 = Write Attributes Response (global command)
        if ( cmdId == 0x01 ) // Read Attributes Response — diagnostic
        {
            const uint8_t *zcl = &af_->data[hdrLen];
            int zclLen = af_->dataLen - hdrLen;
            // ZCL Read Attr Resp: AttrID(2) + Status(1) [+ DataType(1) + Value(variable)]
            if ( zclLen >= 4 && zcl[0] == 0x00 && zcl[1] == 0x80 ) // Attr 0x8000
            {
                uint8_t status = zcl[2];
                if ( status == 0x00 && zclLen >= 5 )
                {
                    uint8_t dataType = zcl[3];
                    printf( "🔍 [DIAGNOSTIC] Onics 0x%04X attr 0x8000: ZCL data type=0x%02X, value bytes:",
                            af_->srcAddr, dataType );
                    for ( int i = 4; i < zclLen; i++ )
                    {
                        printf( " 0x%02X", zcl[i] );
                    }
                    printf( "\n" );
                    if ( zclLen >= 6 )
                    {
                        uint16_t val = (uint16_t)(zcl[4] | (zcl[5] << 8));
                        printf( "   -> Current value: 0x%04X (%s)\n", val,
                                val == 0xFFFF ? "IAS_ZONE_DISABLED" :
                                val == 0x002C ? "PERSONAL_EMERGENCY_DEVICE" : "other" );
                    }
                }
                else
                {
                    printf( "🔍 [DIAGNOSTIC] Onics 0x%04X attr 0x8000 read failed (status=0x%02X)\n",
                            af_->srcAddr, status );
                }
            }
        }
        else if ( cmdId == 0x04 ) // Write Attributes Response
        {
            const uint8_t *zcl = &af_->data[hdrLen];
            int zclLen = af_->dataLen - hdrLen;
            if ( zclLen == 0 || ( zclLen >= 1 && zcl[0] == 0x00 ) )
            {
                // Success: either empty payload OR a status=0x00 record.
                // SBTZB-110 sends a 1-byte payload (0x00) on success.
                // The device WILL reset immediately after this to apply the panic mode change.
                // Steps 3 & 4 will be triggered once the device re-pairs with EP 0x23 visible.
                printf( "✅ Onics Button 0x%04X Panic mode activation SUCCESS!\n", af_->srcAddr );
                printf( "   -> EP 0x23 (IAS Zone Panic) is now unlocked in hardware.\n" );
                printf( "   -> Device will reset to apply mode change. Binding EP 0x23 now...\n" );

                // Step 3: Bind IAS Zone cluster (0x0500) on the newly unlocked EP 0x23.
                pthread_mutex_lock( &g_deviceMutex );
                uint8_t ieee[8];
                bool hasIeee = false;
                for ( int i = 0; i < g_numOnicsButtons; i++ )
                {
                    if ( g_onicsButtons[i].shortAddr == af_->srcAddr && g_onicsButtons[i].hasIeee )
                    {
                        memcpy( ieee, g_onicsButtons[i].ieee, 8 );
                        hasIeee = true;
                        break;
                    }
                }
                pthread_mutex_unlock( &g_deviceMutex );

                if ( hasIeee )
                {
                    ZNP_ZdoBindReq( af_->srcAddr, ieee, 0x23, 0x0500, g_coordinatorIeee, 8 );
                    usleep( 500000 );
                    // Step 4: Write coordinator's IEEE to IAS_CIE_Address (0x0010) on EP 0x23.
                    ZNP_WriteCieAddress( af_->srcAddr, 0x23, 0x14 );
                    printf( "   -> CIE Address written to EP 0x23. Waiting for Zone Enroll Request...\n" );
                }
                else
                {
                    printf( "   ⚠️  IEEE not yet known - will retry on re-pair.\n" );
                }
            }
            else if ( zclLen >= 3 && zcl[0] != 0x00 )
            {
                printf( "❌ Onics Button 0x%04X Panic activation FAILED (attr=0x%02X%02X, status=0x%02X)\n",
                        af_->srcAddr, zcl[2], zcl[1], zcl[0] );
            }
        }
    }

    if ((fc & 0x10) == 0 && cmdId != 0x0B) {
        uint8_t transSeq = af_->data[1];
        ZNP_SendDefaultResponse(af_->srcAddr, af_->srcEp, af_->clusterId, transSeq, cmdId, 0x00);
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

    printf( "Configuring Onics SBTZB-110 button 0x%04X...\n", shortAddr_ );

    // Per SBTZB-110 Technical Manual Sections 3.3 & 4.2.3.2:
    // Step 1: Bind On/Off cluster (0x0006) on EP 0x20.
    // Even though we only want IAS Zone alarms on EP 0x23, the hardware seems
    // to stop reporting clicks entirely if this primary output is unbound.
    // We will bind it to keep the button happy, but ignore the 0x0006 messages
    // in the AF handler so they don't double-trigger the siren.
    ZNP_ZdoBindReq( shortAddr_, buttonIeee, endpoint, 0x0006, g_coordinatorIeee, 8 );
    usleep( 500000 );

    // Step 2: Write attr 0x8000 (Uint16) = 0x002C (PERSONAL_EMERGENCY_DEVICE) to
    //         Binary Input cluster (0x000F) on EP 0x20 to enable the hidden EP 0x23
    //         IAS Zone Panic endpoint in the hardware firmware.
    ZNP_SendButtonActivation( shortAddr_, endpoint, 0x13 );
    usleep( 1500000 ); // Wait 1.5s for hardware to unlock EP 0x23

    // Steps 3 & 4 are completed in OnicsButton_HandleAf when the 0x000F Write
    // Attributes Response arrives with status=0x00 (success). On success, we
    // trigger a re-discovery to find EP 0x23, bind its 0x0500 cluster, and
    // write the CIE address so Zone Enroll Request can proceed.

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
        pthread_mutex_lock( &g_deviceMutex );
        for ( int i = 0; i < g_numOnicsButtons; i++ )
        {
            if ( g_onicsButtons[i].shortAddr == shortAddr_ )
            {
                g_onicsButtons[i].lastPanicTime = ZNP_GetCurrentTime();
                break;
            }
        }
        pthread_mutex_unlock( &g_deviceMutex );
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

void OnicsButton_ReadEnvironment( uint16_t shortAddr_ )
{
    printf( "Requesting Environment Data (Battery) from Onics Button 0x%04X...\n", shortAddr_ );
    
    // Read Battery Voltage (Cluster 0x0001, Attr 0x0020) on EP 0x20
    uint8_t zclFrameBat[5];
    zclFrameBat[0] = 0x00; 
    zclFrameBat[1] = 0xD3;
    zclFrameBat[2] = 0x00; // Read Attributes
    zclFrameBat[3] = 0x20; // Attr 0x0020
    zclFrameBat[4] = 0x00; 
    ZNP_AfDataRequestExt( 2, shortAddr_, 0x20, 0, 8, 0x0001, 0xD3, 0, 30, zclFrameBat, 5 );
}
#endif
