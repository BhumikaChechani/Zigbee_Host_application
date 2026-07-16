#include "config.h"
#if ENABLE_AQARA_OCCUPANCY
///
/// @file   aqara_occupancy.c
/// @brief  Implementation of the Aqara occupancy sensor module (see aqara_occupancy.h).
///
#include "aqara_occupancy.h"
#include "msg_queue.h"
#include "usecase.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

AQARA_OCCUPANCY_T g_aqaraOccupancies[MAX_AQARA_OCCUPANCY];
int g_numAqaraOccupancies = 0;

static MSG_QUEUE_T s_occupancyInbox;
static pthread_t s_occupancyThread;
static uint16_t s_lightThreshold = 10000;

///
/// @brief  Fixed byte width of a ZCL data type.
/// @param  type_  ZCL data type id.
/// @return Byte width for fixed-width types, or -1 for variable/unknown types
///         (strings, or types we don't model) so the caller stops parsing.
///
static int AqaraOccupancy_ZclTypeLen( uint8_t type_ )
{
    switch ( type_ )
    {
        case 0x00: return 0;                                     // no data
        case 0x10: case 0x18: case 0x20: case 0x28: case 0x30:
            return 1;                                           // bool/map8/uint8/int8/enum8
        case 0x19: case 0x21: case 0x29: case 0x31:
            return 2;                                           // map16/uint16/int16/enum16
        case 0x1A: case 0x22: case 0x2A:
            return 3;                                           // 24-bit
        case 0x1B: case 0x23: case 0x2B: case 0x39:
            return 4;                                           // 32-bit / single float
        case 0x25: case 0x26:
            return 6;                                           // 48-bit
        case 0x27: case 0x2F: case 0x3A:
            return 8;                                           // 64-bit / double
        default: return -1;                                    // strings (0x41/0x42) / unknown
    }
}

///
/// @brief  Parse a presence report AF frame and forward it as a use-case event.
///
/// Accepts the Aqara manufacturer-specific cluster 0xFCC0 (attribute 0x0142,
/// which is how this sensor actually reports presence) and the standard
/// Occupancy Sensing cluster 0x0406 (attribute 0x0000) for other models.
///
/// @param  af_  The decoded AF message (cluster 0xFCC0 or 0x0406).
/// @return None.
///
static void AqaraOccupancy_HandleAf( const AF_MSG_T *af_ )
{
    if ( ( af_->clusterId != 0x0406 && af_->clusterId != 0xFCC0 && af_->clusterId != 0x0402 && af_->clusterId != 0x0405 && af_->clusterId != 0x0400 ) || af_->dataLen < 5 )
    {
        return;
    }
    uint8_t fc = af_->data[0];
    int hdrLen = ( fc & 0x04 ) ? 5 : 3; // check if manufacturer code is present (FC bit 2)
    if ( af_->dataLen < hdrLen )
    {
        return;
    }
    uint8_t cmdId = af_->data[hdrLen - 1];
    if ( cmdId == 0x0A || cmdId == 0x01 ) // Report Attributes or Read Attributes Response
    {
        int offset = hdrLen;
        if ( cmdId == 0x01 || cmdId == 0x0A )
        {
            while ( offset + (cmdId == 0x01 ? 4 : 3) <= af_->dataLen )
            {
                uint16_t attrId = af_->data[offset] | ( af_->data[offset + 1] << 8 );
                uint8_t status = 0;
                uint8_t dataType = 0;
                if ( cmdId == 0x01 )
                {
                    status = af_->data[offset + 2];
                    if ( status != 0 )
                    {
                        printf( "   [OCC] cluster=0x%04X attr=0x%04X status=0x%02X\n", af_->clusterId, attrId, status );
                        offset += 3;
                        continue;
                    }
                    dataType = af_->data[offset + 3];
                    offset += 4;
                }
                else
                {
                    dataType = af_->data[offset + 2];
                    offset += 3;
                }

                // Parse the attribute ID and data type

                // Octet/character strings carry a 1-byte length prefix. The FP300
                // heartbeat (attr 0x00F7) is a packed octet string; skip it whole
                // so we stay aligned on the following attributes.
                if ( dataType == 0x41 || dataType == 0x42 )
                {
                    if ( offset >= af_->dataLen ) break;
                    int slen = af_->data[offset++];
                    offset += slen;
                    continue;
                }

                int width = AqaraOccupancy_ZclTypeLen( dataType );
                if ( width < 0 || offset + width > af_->dataLen )
                {
                    printf( "   [OCC] unhandled type 0x%02X (attr 0x%04X) - stopping parse\n",
                            dataType, attrId );
                    break;
                }

                if ( af_->clusterId == 0x0402 && attrId == 0x0000 && width == 2 )
                {
                    int16_t temp = (int16_t)(af_->data[offset] | ( af_->data[offset + 1] << 8 ));
                    printf( "🌡️ [OCC] Temperature from 0x%04X: %.2f °C\n", af_->srcAddr, temp / 100.0 );
                }
                else if ( af_->clusterId == 0x0405 && attrId == 0x0000 && width == 2 )
                {
                    uint16_t hum = (uint16_t)(af_->data[offset] | ( af_->data[offset + 1] << 8 ));
                    printf( "💧 [OCC] Humidity from 0x%04X: %.2f %%\n", af_->srcAddr, hum / 100.0 );
                }
                else if ( af_->clusterId == 0x0400 && attrId == 0x0000 && width == 2 )
                {
                    uint16_t light = (uint16_t)(af_->data[offset] | ( af_->data[offset + 1] << 8 ));
                    AqaraOccupancy_HandleLightState( af_->srcAddr, light );
                }
                else if ( attrId == 0x0142 || attrId == 0x0000 ) // FP300 presence / std occupancy
                {
                    AqaraOccupancy_HandleState( af_->srcAddr, af_->data[offset] );
                }
                else if ( attrId == 0x014D ) // FP300 PIR motion
                {
                    printf( "🚶 [OCC] MOTION from 0x%04X: %d\n", af_->srcAddr, af_->data[offset] );
                }
                else if ( attrId == 0x015F && width == 4 ) // FP300 target distance (cm)
                {
                    uint32_t cm = af_->data[offset] | ( af_->data[offset + 1] << 8 ) |
                                  ( af_->data[offset + 2] << 16 ) | ( (uint32_t)af_->data[offset + 3] << 24 );
                    AqaraOccupancy_HandleDistance( af_->srcAddr, cm );
                }
                else
                {
                    if ( af_->clusterId == 0xFCC0 )
                    {
                        printf( "   [OCC] unhandled 0xFCC0 attr=0x%04X type=0x%02X\n", attrId, dataType );
                    }
                }

                offset += width;
            }
        }
    }
}

void AqaraOccupancy_PollAll( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    int num = g_numAqaraOccupancies;
    int validNum = 0;
    uint16_t addrs[MAX_AQARA_OCCUPANCY];
    uint8_t eps[MAX_AQARA_OCCUPANCY];
    bool rawOcc[MAX_AQARA_OCCUPANCY];
    for ( int i = 0; i < num; i++ )
    {
        if ( !g_aqaraOccupancies[i].configured ) continue;
        addrs[validNum] = g_aqaraOccupancies[i].shortAddr;
        eps[validNum] = g_aqaraOccupancies[i].endpoint;
        rawOcc[validNum] = g_aqaraOccupancies[i].rawOccupied;
        validNum++;
    }
    pthread_mutex_unlock( &g_deviceMutex );

    if ( validNum == 0 ) return;
    num = validNum;

    static uint8_t zclSeq = 100; // distinct sequence range
    
    for ( int i = 0; i < num; i++ )
    {
        if ( rawOcc[i] )
        {
            // Trigger distance tracking BEFORE polling so the read returns fresh data
            uint8_t writeZcl[9] = { 0x04, 0x5F, 0x11, ++zclSeq, 0x02, 0x98, 0x01, 0x20, 0x01 };
            ZNP_AfDataRequestExt( 0x02, addrs[i], eps[i], 0x0000, 8, 0xFCC0, zclSeq, 0x00, 0x1E, writeZcl, 9 );
            usleep( 50000 );
        }

        uint8_t readZcl[9] = { 0x04, 0x5F, 0x11, ++zclSeq, 0x00, 0x42, 0x01, 0x5F, 0x01 };
        // Poll Presence & Distance
        ZNP_AfDataRequestExt( 0x02, addrs[i], eps[i], 0x0000, 8, 0xFCC0, zclSeq, 0x00, 0x1E, readZcl, 9 );
        usleep( 100000 );
        
        uint8_t readLhtZcl[5] = { 0x00, ++zclSeq, 0x00, 0x00, 0x00 };
        // Poll Light
        ZNP_AfDataRequestExt( 0x02, addrs[i], eps[i], 0x0000, 8, 0x0400, zclSeq, 0x00, 0x1E, readLhtZcl, 5 );
        usleep( 100000 );
    }
}

static void *AqaraOccupancy_PollThread( void *arg_ )
{
    (void)arg_;
    while ( 1 )
    {
        sleep( 2 ); // Check presence/light every 2 seconds
        AqaraOccupancy_PollAll();
    }
    return NULL;
}

///
/// @brief  Aqara occupancy worker thread: run setup on ASSIGN, parse reports on AF.
/// @param  arg_  Unused thread argument.
/// @return NULL (runs until process exit).
///
static void *AqaraOccupancy_Thread( void *arg_ )
{
    (void)arg_;
    while ( 1 )
    {
        SENSOR_MSG_T *msg = (SENSOR_MSG_T *)MsgQueue_Pop( &s_occupancyInbox );
        if ( msg == NULL )
        {
            continue;
        }
        if ( msg->kind == SENSOR_MSG_ASSIGN )
        {
            AqaraOccupancy_Setup( msg->shortAddr );
        }
        else if ( msg->kind == SENSOR_MSG_AF )
        {
            AqaraOccupancy_HandleAf( &msg->af );
        }
        free( msg );
    }
    return NULL;
}

void AqaraOccupancy_Init( void )
{
    memset( g_aqaraOccupancies, 0, sizeof( g_aqaraOccupancies ) );
    g_numAqaraOccupancies = 0;
    MsgQueue_Init( &s_occupancyInbox );
}

void AqaraOccupancy_Start( void )
{
    pthread_create( &s_occupancyThread, NULL, AqaraOccupancy_Thread, NULL );
    
    pthread_t pollThread;
    pthread_create( &pollThread, NULL, AqaraOccupancy_PollThread, NULL );
    pthread_detach( pollThread );
}

void AqaraOccupancy_PostAssign( uint16_t shortAddr_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_ASSIGN;
    msg->shortAddr = shortAddr_;
    MsgQueue_Push( &s_occupancyInbox, msg );
}

void AqaraOccupancy_PostAf( uint16_t shortAddr_, const AF_MSG_T *af_ )
{
    SENSOR_MSG_T *msg = (SENSOR_MSG_T *)calloc( 1, sizeof( SENSOR_MSG_T ) );
    if ( msg == NULL )
    {
        return;
    }
    msg->kind = SENSOR_MSG_AF;
    msg->shortAddr = shortAddr_;
    msg->af = *af_;
    MsgQueue_Push( &s_occupancyInbox, msg );
}

void AqaraOccupancy_Discover( uint16_t shortAddr_, uint8_t endpoint_ )
{
    // Resolve the device's IEEE first. Aqara sleepy sensors frequently rejoin
    // with a brand-new short address, so the IEEE - not the short address - is
    // the stable identity. Matching on it collapses rejoins into one entry
    // instead of accumulating phantom duplicates.
    uint8_t ieee[8];
    bool haveIeee = Device_GetDiscoveredIeee( shortAddr_, ieee );

    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
        if ( haveIeee && g_aqaraOccupancies[i].hasIeee &&
             memcmp( g_aqaraOccupancies[i].ieee, ieee, 8 ) == 0 )
        {
            idx = i;
            break;
        }
    }

    bool changed = false;
    bool isRejoin = false;
    if ( idx == -1 )
    {
        if ( g_numAqaraOccupancies < MAX_AQARA_OCCUPANCY )
        {
            printf( " Aqara Occupancy Sensor discovered: short=0x%04X, ep=0x%02X\n", shortAddr_, endpoint_ );
            g_aqaraOccupancies[g_numAqaraOccupancies].shortAddr = shortAddr_;
            g_aqaraOccupancies[g_numAqaraOccupancies].endpoint = endpoint_;
            g_aqaraOccupancies[g_numAqaraOccupancies].lastSeen = ZNP_GetCurrentTime();
            g_aqaraOccupancies[g_numAqaraOccupancies].hasIeee = haveIeee;
            if ( haveIeee )
            {
                memcpy( g_aqaraOccupancies[g_numAqaraOccupancies].ieee, ieee, 8 );
            }
            g_aqaraOccupancies[g_numAqaraOccupancies].configured = false;
            memset(g_aqaraOccupancies[g_numAqaraOccupancies].zones, 0, sizeof(g_aqaraOccupancies[g_numAqaraOccupancies].zones));
            g_aqaraOccupancies[g_numAqaraOccupancies].zones[0].isActive = true;
            g_aqaraOccupancies[g_numAqaraOccupancies].zones[0].minCm = 0;
            g_aqaraOccupancies[g_numAqaraOccupancies].zones[0].maxCm = 600;
            g_numAqaraOccupancies++;
            changed = true;
        }
    }
    else
    {
        // Known device (possibly rejoining under a new short address).
        if ( g_aqaraOccupancies[idx].shortAddr != shortAddr_ )
        {
            printf( " Aqara Occupancy 0x%04X rejoined as 0x%04X (same IEEE) - reusing entry\n",
                    g_aqaraOccupancies[idx].shortAddr, shortAddr_ );
            g_aqaraOccupancies[idx].shortAddr = shortAddr_;
            isRejoin = true;
            changed = true;
        }
        if ( g_aqaraOccupancies[idx].endpoint != endpoint_ )
        {
            g_aqaraOccupancies[idx].endpoint = endpoint_;
            changed = true;
        }
        g_aqaraOccupancies[idx].lastSeen = ZNP_GetCurrentTime();
        if ( !g_aqaraOccupancies[idx].hasIeee && haveIeee )
        {
            memcpy( g_aqaraOccupancies[idx].ieee, ieee, 8 );
            g_aqaraOccupancies[idx].hasIeee = true;
            changed = true;
        }
    }

    // Device-side bindings target the coordinator's (stable) IEEE, so they
    // survive a rejoin. Re-running the full bind/configure flood on every rejoin
    // only hammers a fragile sleepy device, so skip setup for a pure rejoin of
    // an already-configured sensor.
    bool alreadyConfigured = ( idx != -1 && g_aqaraOccupancies[idx].configured );
    pthread_mutex_unlock( &g_deviceMutex );

    if ( changed )
    {
        Device_Save();
    }
    if ( !( isRejoin && alreadyConfigured ) )
    {
        AqaraOccupancy_PostAssign( shortAddr_ );
    }
}

void AqaraOccupancy_UpdateIeee( uint16_t shortAddr_, const uint8_t *ieee_ )
{
    bool found = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            memcpy( g_aqaraOccupancies[i].ieee, ieee_, 8 );
            g_aqaraOccupancies[i].hasIeee = true;
            found = true;
            break;
        }
    }
    if ( found )
    {
        // Collapse stale duplicates
        for ( int i = g_numAqaraOccupancies - 1; i >= 0; i-- )
        {
            if ( g_aqaraOccupancies[i].shortAddr != shortAddr_ && g_aqaraOccupancies[i].hasIeee &&
                 memcmp( g_aqaraOccupancies[i].ieee, ieee_, 8 ) == 0 )
            {
                for ( int j = i; j < g_numAqaraOccupancies - 1; j++ )
                {
                    g_aqaraOccupancies[j] = g_aqaraOccupancies[j + 1];
                }
                g_numAqaraOccupancies--;
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    if ( found )
    {
        Device_Save();
        AqaraOccupancy_PostAssign( shortAddr_ );
    }
}

void AqaraOccupancy_Setup( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
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
    if ( g_aqaraOccupancies[idx].configured )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    if ( !g_aqaraOccupancies[idx].hasIeee )
    {
        g_aqaraOccupancies[idx].hasIeee = Device_GetDiscoveredIeee( shortAddr_, g_aqaraOccupancies[idx].ieee );
    }
    if ( !g_aqaraOccupancies[idx].hasIeee )
    {
        pthread_mutex_unlock( &g_deviceMutex );
        printf( "Aqara occupancy 0x%04X missing IEEE - requesting...\n", shortAddr_ );
        uint8_t reqPay[4] = { shortAddr_ & 0xFF, ( shortAddr_ >> 8 ) & 0xFF, 0x01, 0x00 };
        ZNP_Sreq( 0x25, 0x01, reqPay, 4, NULL, 3000 );
        return;
    }

    uint8_t sensorIeee[8];
    uint8_t endpoint = g_aqaraOccupancies[idx].endpoint;
    memcpy( sensorIeee, g_aqaraOccupancies[idx].ieee, 8 );
    g_aqaraOccupancies[idx].configured = true;
    pthread_mutex_unlock( &g_deviceMutex );

    printf( "Configuring Aqara occupancy 0x%04X...\n", shortAddr_ );

    // -------------------------------------------------------------------------
    // STEP 1: Bind FCC0 cluster so unsolicited reports reach the coordinator
    // -------------------------------------------------------------------------
    bool bStd = ZNP_ZdoBindReq( shortAddr_, sensorIeee, endpoint, 0x0406, g_coordinatorIeee, 8 );
    bool bMfr = ZNP_ZdoBindReq( shortAddr_, sensorIeee, endpoint, 0xFCC0, g_coordinatorIeee, 8 );
    bool bMs  = ZNP_ZdoBindReq( shortAddr_, sensorIeee, endpoint, 0x0012, g_coordinatorIeee, 8 );
    bool bLht = ZNP_ZdoBindReq( shortAddr_, sensorIeee, endpoint, 0x0400, g_coordinatorIeee, 8 );
    printf( "   [OCC] bind: 0x0406=%s 0xFCC0=%s 0x0012=%s 0x0400=%s\n",
            bStd ? "OK" : "FAIL", bMfr ? "OK" : "FAIL", bMs ? "OK" : "FAIL", bLht ? "OK" : "FAIL" );

    // Helper macro: ZCL frame header (mfr-specific, cluster 0xFCC0, mfr code 0x115F)
    // [FC=0x04][MFR_LO=0x5F][MFR_HI=0x11][SEQ][CMD] ...
    uint8_t seq = 0x01;

    // -------------------------------------------------------------------------
    // STEP 2: Read Presence (0x0142) to wake the device up
    // -------------------------------------------------------------------------
    {
        uint8_t f[7] = { 0x04, 0x5F, 0x11, seq++, 0x00, 0x42, 0x01 }; // Read Attr 0x0142
        ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, seq, 0x00, 0x1E, f, 7 );
    }

    // -------------------------------------------------------------------------
    // STEP 3: Configure Reporting for Presence (0x0142) - uint8, change=1, max=60s
    // -------------------------------------------------------------------------
    {
        uint8_t f[14] = {
            0x04, 0x5F, 0x11, seq++, 0x06,  // ZCL hdr: Configure Reporting
            0x00,                             // direction: send reports
            0x42, 0x01,                       // attr 0x0142
            0x20,                             // data type uint8
            0x00, 0x00,                       // min interval = 0 s
            0x3C, 0x00,                       // max interval = 60 s
            0x01                              // reportable change = 1
        };
        ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, seq, 0x00, 0x1E, f, 14 );
    }

    // -------------------------------------------------------------------------
    // STEP 4: Enable target distance tracking (attr 0x0198 = 408, write 1)
    //   This is REQUIRED - without it the sensor never pushes 0x015F updates.
    // -------------------------------------------------------------------------
    {
        uint8_t f[9] = {
            0x04, 0x5F, 0x11, seq++, 0x02,  // Write Attributes
            0x98, 0x01,                       // attr 0x0198
            0x20,                             // uint8
            0x01                              // value = 1 (start tracking)
        };
        ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, seq, 0x00, 0x1E, f, 9 );
    }

    // -------------------------------------------------------------------------
    // STEP 5: Configure Reporting for Distance (0x015F) - uint32, change=1
    // -------------------------------------------------------------------------
    {
        uint8_t f[17] = {
            0x04, 0x5F, 0x11, seq++, 0x06,  // Configure Reporting
            0x00,                             // direction
            0x5F, 0x01,                       // attr 0x015F
            0x23,                             // uint32
            0x00, 0x00,                       // min = 0 s
            0x05, 0x00,                       // max = 5 s
            0x01, 0x00, 0x00, 0x00            // change = 1
        };
        ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, seq, 0x00, 0x1E, f, 17 );
    }

    // -------------------------------------------------------------------------
    // STEP 6: Set Absence Delay Timer (0x0197) = 10s (minimum supported)
    // -------------------------------------------------------------------------
    {
        uint8_t f[12] = {
            0x04, 0x5F, 0x11, seq++, 0x02,  // Write Attributes
            0x97, 0x01,                       // attr 0x0197
            0x23,                             // uint32
            10, 0x00, 0x00, 0x00              // 10 seconds LE
        };
        ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, seq, 0x00, 0x1E, f, 12 );
    }

    // -------------------------------------------------------------------------
    // STEP 7: Reset Hardware Detection Range to Default (0-600cm = 0xFFFFFF)
    // -------------------------------------------------------------------------
    AqaraOccupancy_SetHwDetectionRange( shortAddr_, 0xFFFFFF );

    printf( "Configuration sent to Aqara occupancy 0x%04X!\n", shortAddr_ );
}

///
/// @brief  Send hardware detection range bitmask to FP300 (attr 0x019A).
///         Each of the 24 bits covers a 25cm slice: bit 0 = 0.00-0.25m, bit 1 = 0.25-0.50m ... bit 23 = 5.75-6.00m
///         Payload format: [prefix_lo][prefix_hi][mask_b0][mask_b1][mask_b2]  (5 bytes, octet string)
///         Default prefix is 0x0300. An all-1s mask (0xFFFFFF) enables the full 6m range.
/// @param  shortAddr_  Sensor network address.
/// @param  bitmask_    24-bit mask of enabled 25cm slices.
///
void AqaraOccupancy_SetHwDetectionRange( uint16_t shortAddr_, uint32_t bitmask_ )
{
    bitmask_ &= 0xFFFFFF; // clamp to 24 bits

    pthread_mutex_lock( &g_deviceMutex );
    uint8_t endpoint = 0x01;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            endpoint = g_aqaraOccupancies[i].endpoint;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    // Build 5-byte octet-string payload: [0x00][0x03][b0][b1][b2]
    uint8_t buf[5];
    buf[0] = 0x00; // prefix low byte
    buf[1] = 0x03; // prefix high byte (0x0300 = default)
    buf[2] = ( bitmask_ ) & 0xFF;
    buf[3] = ( bitmask_ >> 8 ) & 0xFF;
    buf[4] = ( bitmask_ >> 16 ) & 0xFF;

    // ZCL Write Attributes frame for attr 0x019A (octet string type 0x41)
    uint8_t f[5 + 5 + 4] = {
        0x04, 0x5F, 0x11, 0x20, 0x02,   // mfr-specific Write Attributes
        0x9A, 0x01,                       // attr 0x019A
        0x41,                             // octet string
        0x05,                             // length = 5 bytes
        buf[0], buf[1], buf[2], buf[3], buf[4]
    };
    ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, 0x21, 0x00, 0x1E, f, sizeof(f) );

    printf( "[OCC] HW detection range bitmask 0x%06X sent to 0x%04X\n", bitmask_, shortAddr_ );
}

///
/// @brief  Trigger AI Spatial Learning on FP300 (attr 0x0157 = 343, write 1).
///         Run this while the room is EMPTY to let the radar learn its static environment.
///
void AqaraOccupancy_SpatialLearning( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    uint8_t endpoint = 0x01;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ ) { endpoint = g_aqaraOccupancies[i].endpoint; break; }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    uint8_t f[9] = { 0x04, 0x5F, 0x11, 0x30, 0x02, 0x57, 0x01, 0x20, 0x01 };
    ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, 0x31, 0x00, 0x1E, f, 9 );
    printf( "[OCC] AI Spatial Learning triggered on 0x%04X - keep room EMPTY for 30s\n", shortAddr_ );
}

///
/// @brief  Set motion sensitivity on FP300 (attr 0x010C).
///         1=low  2=medium  3=high
///
void AqaraOccupancy_SetSensitivity( uint16_t shortAddr_, uint8_t level_ )
{
    if ( level_ < 1 || level_ > 3 ) { printf( "Sensitivity: 1=low 2=medium 3=high\n" ); return; }

    pthread_mutex_lock( &g_deviceMutex );
    uint8_t endpoint = 0x01;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ ) { endpoint = g_aqaraOccupancies[i].endpoint; break; }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    const char *labels[] = { "", "low", "medium", "high" };
    uint8_t f[9] = { 0x04, 0x5F, 0x11, 0x32, 0x02, 0x0C, 0x01, 0x20, level_ };
    ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0, 0x33, 0x00, 0x1E, f, 9 );
    printf( "Configuring Aqara Occupancy Sensor 0x%04X sensitivity to level %d (%s)...\n", shortAddr_, level_, labels[level_] );
}

void AqaraOccupancy_ReadEnvironment( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    uint8_t endpoint = 0x01;
    bool found = false;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ ) { endpoint = g_aqaraOccupancies[i].endpoint; found = true; break; }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    if ( !found )
    {
        printf( "Device 0x%04X not registered as an occupancy sensor.\n", shortAddr_ );
        return;
    }

    static uint8_t zclSeq = 50;
    
    printf( "Fetching environment data from 0x%04X...\n", shortAddr_ );
    
    // Read Temperature (0x0402)
    uint8_t readTmpZcl[5] = { 0x00, ++zclSeq, 0x00, 0x00, 0x00 };
    ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0x0402, zclSeq, 0x00, 0x1E, readTmpZcl, 5 );
    
    usleep( 250000 );
    
    // Read Humidity (0x0405)
    uint8_t readHumZcl[5] = { 0x00, ++zclSeq, 0x00, 0x00, 0x00 };
    ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0x0405, zclSeq, 0x00, 0x1E, readHumZcl, 5 );
}

static void EvaluatePresenceLogic( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
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

    bool sensorSaysOccupied = g_aqaraOccupancies[idx].rawOccupied;
    uint32_t dist = g_aqaraOccupancies[idx].currentDistanceCm;
    
    // Evaluate each active zone
    for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
    {
        if ( !g_aqaraOccupancies[idx].zones[z].isActive ) continue;
        
        bool logicalOccupied;

        if ( !sensorSaysOccupied )
        {
            // Sensor explicitly says NO presence -> always clear all zones immediately.
            logicalOccupied = false;
        }
        else if ( dist == 0 )
        {
            // Fall back to raw presence only if zone starts at 0.
            logicalOccupied = ( g_aqaraOccupancies[idx].zones[z].minCm == 0 );
        }
        else
        {
            // Valid distance -> apply software zone filter.
            logicalOccupied = ( dist >= g_aqaraOccupancies[idx].zones[z].minCm &&
                                dist <= g_aqaraOccupancies[idx].zones[z].maxCm );
        }
        bool stateChanged = ( logicalOccupied != g_aqaraOccupancies[idx].zones[z].occupied );
        
        g_aqaraOccupancies[idx].zones[z].occupied = logicalOccupied;
        
        if ( stateChanged )
        {
            if ( logicalOccupied )
            {
                printf( "   ✅ PRESENCE DETECTED on sensor 0x%04X (distance %u cm is in zone %d [%u-%u])\n", 
                        shortAddr_, dist, z, g_aqaraOccupancies[idx].zones[z].minCm, g_aqaraOccupancies[idx].zones[z].maxCm );
                
                // Pack zone index into the 'raw' argument of UC_EVT_T so usecase knows which zone triggered
                UseCase_Post( UC_OCCUPANCY_DETECTED, shortAddr_, z );
            }
            else
            {
                if ( sensorSaysOccupied && dist > 0 )
                    printf( "   🚫 PRESENCE IGNORED on sensor 0x%04X (distance %u cm outside zone %d [%u-%u])\n", 
                            shortAddr_, dist, z, g_aqaraOccupancies[idx].zones[z].minCm, g_aqaraOccupancies[idx].zones[z].maxCm );
                else
                    printf( "   ❌ PRESENCE CLEARED on sensor 0x%04X (zone %d)\n", shortAddr_, z );
                    
                UseCase_Post( UC_OCCUPANCY_CLEARED, shortAddr_, z );
            }
        }
    }
    
    pthread_mutex_unlock( &g_deviceMutex );
}

void AqaraOccupancy_HandleState( uint16_t shortAddr_, uint8_t occupied_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    uint8_t endpoint = 0x01;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            g_aqaraOccupancies[i].rawOccupied = ( occupied_ != 0 );
            g_aqaraOccupancies[i].lastSeen = ZNP_GetCurrentTime();
            endpoint = g_aqaraOccupancies[i].endpoint;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    if ( occupied_ != 0 )
    {
        // The sensor just sent us a presence report -> it is guaranteed awake RIGHT NOW.
        // Trigger distance tracking (attr 0x0198 = 1) while the radio is still active.
        // The sensor will respond with unsolicited 0x015F (distance) reports so that
        // EvaluatePresenceLogic can apply zone filtering with a real distance value.
        static uint8_t s_distSeq = 0xD0;
        uint8_t f[9] = { 0x04, 0x5F, 0x11, ++s_distSeq, 0x02,
                         0x98, 0x01,   // attr 0x0198
                         0x20,         // uint8
                         0x01 };       // value = 1 (start tracking)
        ZNP_AfDataRequestExt( 0x02, shortAddr_, endpoint, 0x0000, 8, 0xFCC0,
                              s_distSeq, 0x00, 0x1E, f, sizeof(f) );
    }

    EvaluatePresenceLogic( shortAddr_ );
}

void AqaraOccupancy_HandleDistance( uint16_t shortAddr_, uint32_t cm_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            g_aqaraOccupancies[i].currentDistanceCm = cm_;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    
    EvaluatePresenceLogic( shortAddr_ );
}

void AqaraOccupancy_SetZone( uint16_t shortAddr_, int zoneIdx_, uint32_t minCm_, uint32_t maxCm_ )
{
    if ( zoneIdx_ < 0 || zoneIdx_ >= MAX_OCCUPANCY_ZONES )
    {
        printf( "Invalid zone index %d (must be 0 to %d).\n", zoneIdx_, MAX_OCCUPANCY_ZONES - 1 );
        return;
    }

    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
    }
    if ( idx != -1 )
    {
        g_aqaraOccupancies[idx].zones[zoneIdx_].isActive = true;
        g_aqaraOccupancies[idx].zones[zoneIdx_].minCm = minCm_;
        g_aqaraOccupancies[idx].zones[zoneIdx_].maxCm = maxCm_;
        printf( "Aqara Occupancy 0x%04X zone %d set to %u - %u cm.\n", shortAddr_, zoneIdx_, minCm_, maxCm_ );
        Device_Save();
        // Zone boundaries enforced in software (EvaluatePresenceLogic + distance filter).
        // Hardware stays at full range (0xFFFFFF) so the radar DSP tracking never breaks.
        pthread_mutex_unlock( &g_deviceMutex );
        EvaluatePresenceLogic( shortAddr_ );
        return;
    }
    pthread_mutex_unlock( &g_deviceMutex );
    EvaluatePresenceLogic( shortAddr_ );
}

void AqaraOccupancy_DeleteZone( uint16_t shortAddr_, int zoneIdx_ )
{
    if ( zoneIdx_ < 0 || zoneIdx_ >= MAX_OCCUPANCY_ZONES )
    {
        printf( "Invalid zone index %d (must be 0 to %d).\n", zoneIdx_, MAX_OCCUPANCY_ZONES - 1 );
        return;
    }
    if ( zoneIdx_ == 0 )
    {
        printf( "Cannot delete zone 0 (it is the default zone). Use 'zone' to change its range.\n" );
        return;
    }

    pthread_mutex_lock( &g_deviceMutex );
    int idx = -1;
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            idx = i;
            break;
        }
    }
    if ( idx != -1 )
    {
        g_aqaraOccupancies[idx].zones[zoneIdx_].isActive = false;
        g_aqaraOccupancies[idx].zones[zoneIdx_].occupied = false;
        printf( "Aqara Occupancy 0x%04X zone %d deleted.\n", shortAddr_, zoneIdx_ );
        Device_Save();
        pthread_mutex_unlock( &g_deviceMutex );
        return;
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

bool AqaraOccupancy_AllOccupied( void )
{
    bool all = false;
    pthread_mutex_lock( &g_deviceMutex );
    if ( g_numAqaraOccupancies > 0 )
    {
        all = true;
        for ( int i = 0; i < g_numAqaraOccupancies; i++ )
        {
            bool sensorOcc = false;
            for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
            {
                if ( g_aqaraOccupancies[i].zones[z].isActive && g_aqaraOccupancies[i].zones[z].occupied )
                {
                    sensorOcc = true;
                    break;
                }
            }
            if ( !sensorOcc )
            {
                all = false;
                break;
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return all;
}

void AqaraOccupancy_HandleLightState( uint16_t shortAddr_, uint16_t light_ )
{
    // Threshold tuning: light > s_lightThreshold implies significant brightness
    bool lightIsOn = ( light_ > s_lightThreshold );
    bool stateChanged = false;

    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            g_aqaraOccupancies[i].lastLightLevel = light_;
            if ( !g_aqaraOccupancies[i].hasLightState || g_aqaraOccupancies[i].isLightOn != lightIsOn )
            {
                g_aqaraOccupancies[i].hasLightState = true;
                g_aqaraOccupancies[i].isLightOn = lightIsOn;
                stateChanged = true;
            }
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );

    if ( stateChanged )
    {
        if ( !lightIsOn )
        {
            printf( "🌙 Light is OFF (0x%04X) [Raw: %u, Threshold: %u]\n", shortAddr_, light_, s_lightThreshold );
            UseCase_Post( UC_LIGHT_OFF, shortAddr_, light_ );
        }
        else
        {
            printf( "☀️ Light is ON (0x%04X) [Raw: %u, Threshold: %u]\n", shortAddr_, light_, s_lightThreshold );
            UseCase_Post( UC_LIGHT_ON, shortAddr_, light_ );
        }
    }
}

int AqaraOccupancy_OccupiedCount( void )
{
    int count = 0;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        bool sensorOcc = false;
        for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
        {
            if ( g_aqaraOccupancies[i].zones[z].isActive && g_aqaraOccupancies[i].zones[z].occupied )
            {
                sensorOcc = true;
                break;
            }
        }
        if ( sensorOcc )
        {
            count++;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return count;
}

void AqaraOccupancy_PrintStatus( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    printf( "Registered Aqara Occupancies (%d):\n", g_numAqaraOccupancies );
    double now = ZNP_GetCurrentTime();
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        printf( "  - 0x%04X: IEEE=", g_aqaraOccupancies[i].shortAddr );
        if ( g_aqaraOccupancies[i].hasIeee )
        {
            for ( int j = 7; j >= 0; j-- )
            {
                printf( "%02x", g_aqaraOccupancies[i].ieee[j] );
            }
        }
        else
        {
            printf( "Unknown" );
        }
        printf( ", ep=0x%02X, last_seen=%.1fs ago\n",
                g_aqaraOccupancies[i].endpoint, now - g_aqaraOccupancies[i].lastSeen );
        if ( g_aqaraOccupancies[i].hasLightState )
        {
            printf( "    Light: %s (Raw: %u, Threshold: %u)\n", 
                    g_aqaraOccupancies[i].isLightOn ? "ON" : "OFF", 
                    g_aqaraOccupancies[i].lastLightLevel, 
                    s_lightThreshold );
        }
        else
        {
            printf( "    Light: Unknown (Threshold: %u)\n", s_lightThreshold );
        }

        int activeZones = 0;
        for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
        {
            if ( g_aqaraOccupancies[i].zones[z].isActive ) activeZones++;
        }
        
        printf( "    Active Zones (%d):\n", activeZones );
        for ( int z = 0; z < MAX_OCCUPANCY_ZONES; z++ )
        {
            if ( g_aqaraOccupancies[i].zones[z].isActive )
            {
                printf( "      - Zone %d: slice %u to %u (%u cm - %u cm)\n", 
                        z, 
                        g_aqaraOccupancies[i].zones[z].minCm / 25, 
                        g_aqaraOccupancies[i].zones[z].maxCm / 25,
                        g_aqaraOccupancies[i].zones[z].minCm,
                        g_aqaraOccupancies[i].zones[z].maxCm );
            }
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

bool AqaraOccupancy_IsKnown( uint16_t shortAddr_ )
{
    bool known = false;
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            known = true;
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
    return known;
}

void AqaraOccupancy_UpdateSeen( uint16_t shortAddr_ )
{
    pthread_mutex_lock( &g_deviceMutex );
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        if ( g_aqaraOccupancies[i].shortAddr == shortAddr_ )
        {
            g_aqaraOccupancies[i].lastSeen = ZNP_GetCurrentTime();
            break;
        }
    }
    pthread_mutex_unlock( &g_deviceMutex );
}

void AqaraOccupancy_DiscoverAllActiveEp( void )
{
    pthread_mutex_lock( &g_deviceMutex );
    int tempNum = g_numAqaraOccupancies;
    uint16_t tempAddrs[MAX_AQARA_OCCUPANCY];
    for ( int i = 0; i < g_numAqaraOccupancies; i++ )
    {
        tempAddrs[i] = g_aqaraOccupancies[i].shortAddr;
    }
    pthread_mutex_unlock( &g_deviceMutex );

    for ( int i = 0; i < tempNum; i++ )
    {
        ZNP_ZdoActiveEpReq( tempAddrs[i] );
        ZNP_QuerySimpleDesc( tempAddrs[i], 1 );
    }
}

void AqaraOccupancy_SetLightThreshold( uint16_t threshold_ )
{
    s_lightThreshold = threshold_;
}

uint16_t AqaraOccupancy_GetLightThreshold( void )
{
    return s_lightThreshold;
}
#endif

