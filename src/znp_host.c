///
/// @file   znp_host.c
/// @brief  Serial transport, MT framing, reader thread, and ZNP command wrappers.
///
/// Implements the interface declared in znp_host.h. See that header for the
/// per-function contracts; this file documents the private (static) helpers.
///
#include "znp_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>

// Global variables declared in the header
IEEE_MAPPING_T g_discoveredIeees[MAX_DISCOVERED_IEEES];
int g_numDiscoveredIeees = 0;

bool g_sirenActive = false;
uint8_t g_nextZoneId = 1;
uint8_t g_coordinatorIeee[8] = { 0 };
bool g_hasCoordinatorIeee = false;


EVENT_QUEUE_T g_eventQueue;

// File descriptor & threading state
static int s_serialFd = -1;
static pthread_t s_readerThread;
static bool s_readerRunning = false;

// SREQ/SRSP synchronization
static pthread_mutex_t s_sreqMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t s_srspMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_srspCond = PTHREAD_COND_INITIALIZER;
static MT_FRAME_T s_srspFrame;
static bool s_srspReceived = false;
static uint8_t s_expectedSrspCmd0 = 0;
static uint8_t s_expectedSrspCmd1 = 0;

// Local declarations
static void *ZNP_ReaderThread( void *arg_ );

// Helper function to get current time in seconds
double ZNP_GetCurrentTime( void )
{
    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts );
    return ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

// ---------------------------------------------------------------------------
// Thread-safe Event Queue Functions
// ---------------------------------------------------------------------------
void EventQueue_Init( EVENT_QUEUE_T *queue_ )
{
    queue_->head = 0;
    queue_->tail = 0;
    queue_->count = 0;
    pthread_mutex_init( &queue_->mutex, NULL );
    pthread_cond_init( &queue_->cond, NULL );
}

void EventQueue_Push( EVENT_QUEUE_T *queue_, const MT_FRAME_T *frame_ )
{
    pthread_mutex_lock( &queue_->mutex );
    if ( queue_->count < EVENT_QUEUE_MAX )
    {
        queue_->frames[queue_->tail] = *frame_;
        queue_->tail = ( queue_->tail + 1 ) % EVENT_QUEUE_MAX;
        queue_->count++;
        pthread_cond_signal( &queue_->cond );
    }
    else
    {
        printf( "⚠️ Event queue overflow, dropping packet cmd0=0x%02X cmd1=0x%02X\n",
                frame_->cmd0, frame_->cmd1 );
    }
    pthread_mutex_unlock( &queue_->mutex );
}

bool EventQueue_Pop( EVENT_QUEUE_T *queue_, MT_FRAME_T *frame_, int timeoutMs_ )
{
    pthread_mutex_lock( &queue_->mutex );

    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts );
    ts.tv_sec += timeoutMs_ / 1000;
    ts.tv_nsec += ( timeoutMs_ % 1000 ) * 1000000;
    if ( ts.tv_nsec >= 1000000000 )
    {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
    }

    while ( queue_->count == 0 )
    {
        int ret = pthread_cond_timedwait( &queue_->cond, &queue_->mutex, &ts );
        if ( ret == ETIMEDOUT )
        {
            pthread_mutex_unlock( &queue_->mutex );
            return false;
        }
    }

    *frame_ = queue_->frames[queue_->head];
    queue_->head = ( queue_->head + 1 ) % EVENT_QUEUE_MAX;
    queue_->count--;

    pthread_mutex_unlock( &queue_->mutex );
    return true;
}

void EventQueue_Clear( EVENT_QUEUE_T *queue_ )
{
    pthread_mutex_lock( &queue_->mutex );
    queue_->head = 0;
    queue_->tail = 0;
    queue_->count = 0;
    pthread_mutex_unlock( &queue_->mutex );
}

// ---------------------------------------------------------------------------
// Serial & MT Core Functions
// ---------------------------------------------------------------------------
bool ZNP_Init( const char *port_ )
{
    s_serialFd = open( port_, O_RDWR | O_NOCTTY | O_NDELAY );
    if ( s_serialFd < 0 )
    {
        perror( "Failed to open serial port" );
        return false;
    }

    struct termios options;
    if ( tcgetattr( s_serialFd, &options ) != 0 )
    {
        perror( "tcgetattr failed" );
        close( s_serialFd );
        s_serialFd = -1;
        return false;
    }

    cfsetispeed( &options, B115200 );
    cfsetospeed( &options, B115200 );

    options.c_cflag |= ( CLOCAL | CREAD );
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS; // Disable HW flow control

    options.c_lflag &= ~( ICANON | ECHO | ECHOE | ISIG ); // Raw input
    options.c_oflag &= ~OPOST; // Raw output
    options.c_iflag &= ~( IXON | IXOFF | IXANY ); // Disable SW flow control

    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    if ( tcsetattr( s_serialFd, TCSANOW, &options ) != 0 )
    {
        perror( "tcsetattr failed" );
        close( s_serialFd );
        s_serialFd = -1;
        return false;
    }

    tcflush( s_serialFd, TCIOFLUSH );

    // The port was opened O_NDELAY so open() would not block on modem-control
    // lines. Clear O_NONBLOCK now so write()/read() block as expected; the
    // reader thread still gates reads with poll(), so it never stalls.
    if ( fcntl( s_serialFd, F_SETFL, 0 ) != 0 )
    {
        perror( "fcntl F_SETFL failed" );
        close( s_serialFd );
        s_serialFd = -1;
        return false;
    }

    EventQueue_Init( &g_eventQueue );

    s_readerRunning = true;
    if ( pthread_create( &s_readerThread, NULL, ZNP_ReaderThread, NULL ) != 0 )
    {
        perror( "Failed to create reader thread" );
        close( s_serialFd );
        s_serialFd = -1;
        s_readerRunning = false;
        return false;
    }

    return true;
}

void ZNP_Close( void )
{
    if ( s_readerRunning )
    {
        s_readerRunning = false;
        pthread_join( s_readerThread, NULL );
    }
    if ( s_serialFd >= 0 )
    {
        close( s_serialFd );
        s_serialFd = -1;
    }
}

uint8_t ZNP_CalcFcs( uint8_t len_, uint8_t cmd0_, uint8_t cmd1_, const uint8_t *payload_ )
{
    uint8_t fcs = len_ ^ cmd0_ ^ cmd1_;
    for ( int i = 0; i < len_; i++ )
    {
        fcs ^= payload_[i];
    }
    return fcs;
}

///
/// @brief  Write the whole buffer, retrying short writes and EINTR.
///
/// The port is in blocking mode (O_NONBLOCK is cleared in ZNP_Init), so this
/// normally completes in a single write() call.
///
/// @param  fd_   Open serial file descriptor.
/// @param  buf_  Bytes to write.
/// @param  len_  Number of bytes.
/// @return true if all bytes were written; false on a hard error.
///
static bool ZNP_WriteAll( int fd_, const uint8_t *buf_, int len_ )
{
    int off = 0;
    while ( off < len_ )
    {
        int written = write( fd_, buf_ + off, len_ - off );
        if ( written < 0 )
        {
            if ( errno == EINTR )
            {
                continue;
            }
            return false;
        }
        off += written;
    }
    return true;
}

bool ZNP_Sreq( uint8_t cmd0_, uint8_t cmd1_, const uint8_t *payload_, uint8_t len_,
               MT_FRAME_T *rxFrame_, int timeoutMs_ )
{
    pthread_mutex_lock( &s_sreqMutex );

    // Set the expected response id and clear the received flag together under
    // s_srspMutex, so the reader thread never observes a half-updated
    // expectation.
    pthread_mutex_lock( &s_srspMutex );
    s_expectedSrspCmd0 = 0x60 | ( cmd0_ & 0x1F );
    s_expectedSrspCmd1 = cmd1_;
    s_srspReceived = false;
    pthread_mutex_unlock( &s_srspMutex );

    // Build packet
    uint8_t txBuf[300];
    txBuf[0] = 0xFE;
    txBuf[1] = len_;
    txBuf[2] = cmd0_;
    txBuf[3] = cmd1_;
    if ( len_ > 0 && payload_ != NULL )
    {
        memcpy( &txBuf[4], payload_, len_ );
    }
    txBuf[4 + len_] = ZNP_CalcFcs( len_, cmd0_, cmd1_, payload_ );

    // NOTE: do NOT flush the input buffer here. The reader thread already
    // matches SRSPs by expected cmd0/cmd1, so a stale response is ignored on
    // its own. Flushing races with (and silently discards) asynchronous AREQ
    // events - joins, button presses, zone notifications - that just arrived.
    int totalWrite = 5 + len_;
    if ( !ZNP_WriteAll( s_serialFd, txBuf, totalWrite ) )
    {
        pthread_mutex_unlock( &s_sreqMutex );
        return false;
    }

    // Wait on the condition variable for the matching SRSP
    struct timespec ts;
    clock_gettime( CLOCK_REALTIME, &ts );
    ts.tv_sec += timeoutMs_ / 1000;
    ts.tv_nsec += ( timeoutMs_ % 1000 ) * 1000000;
    if ( ts.tv_nsec >= 1000000000 )
    {
        ts.tv_sec += 1;
        ts.tv_nsec -= 1000000000;
    }

    bool success = false;
    pthread_mutex_lock( &s_srspMutex );
    while ( !s_srspReceived )
    {
        int ret = pthread_cond_timedwait( &s_srspCond, &s_srspMutex, &ts );
        if ( ret == ETIMEDOUT )
        {
            break;
        }
    }
    if ( s_srspReceived )
    {
        if ( rxFrame_ != NULL )
        {
            *rxFrame_ = s_srspFrame;
        }
        success = true;
    }
    pthread_mutex_unlock( &s_srspMutex );

    pthread_mutex_unlock( &s_sreqMutex );
    return success;
}

///
/// @brief  Serial reader thread: frame the MT byte stream and route each frame.
///
/// Continuously polls the port, accumulates bytes, validates each frame's FCS,
/// and dispatches: SRSP frames satisfy the pending ZNP_Sreq() waiter, while
/// AREQ indications are pushed onto ::g_eventQueue for the dispatcher.
///
/// @param  arg_  Unused thread argument.
/// @return NULL when the reader is stopped.
///
static void *ZNP_ReaderThread( void *arg_ )
{
    (void)arg_;
    uint8_t readBuf[256];
    uint8_t parseBuf[1024];
    int parseLen = 0;

    struct pollfd pfd;
    pfd.fd = s_serialFd;
    pfd.events = POLLIN;

    while ( s_readerRunning )
    {
        int pollRet = poll( &pfd, 1, 10 ); // 10 ms poll timeout
        if ( pollRet < 0 )
        {
            if ( errno == EINTR )
            {
                continue; // interrupted by a signal, not a real error
            }
            printf( "❌ [SERIAL] poll() failed (%s) - reader thread exiting; ZNP link lost\n",
                    strerror( errno ) );
            break;
        }
        if ( pollRet == 0 )
        {
            continue;
        }

        int n = read( s_serialFd, readBuf, sizeof( readBuf ) );
        if ( n < 0 )
        {
            if ( errno == EAGAIN || errno == EINTR )
            {
                continue;
            }
            printf( "❌ [SERIAL] read() error (%s) - reader thread exiting; ZNP link lost\n",
                    strerror( errno ) );
            break;
        }
        if ( n == 0 )
        {
            // EOF: the serial device went away (unplugged / USB re-enumerated on
            // a reset). Nothing more will ever arrive on this fd, so the app
            // would appear frozen - make that visible instead of hanging silently.
            printf( "❌ [SERIAL] EOF on serial port - device disconnected; reader thread exiting\n" );
            break;
        }

        // Overflow protection
        if ( parseLen + n > (int)sizeof( parseBuf ) )
        {
            parseLen = 0;
        }
        memcpy( &parseBuf[parseLen], readBuf, n );
        parseLen += n;

        int idx = 0;
        while ( idx < parseLen )
        {
            if ( parseBuf[idx] != 0xFE )
            {
                idx++;
                continue;
            }

            if ( idx + 4 > parseLen )
            {
                break; // Wait for header
            }

            uint8_t len = parseBuf[idx + 1];
            uint8_t cmd0 = parseBuf[idx + 2];
            uint8_t cmd1 = parseBuf[idx + 3];

            int frameSize = 5 + len;
            if ( idx + frameSize > parseLen )
            {
                break; // Wait for full payload + FCS
            }

            uint8_t fcs = parseBuf[idx + 4 + len];
            uint8_t calc = ZNP_CalcFcs( len, cmd0, cmd1, &parseBuf[idx + 4] );

            if ( fcs == calc )
            {
                MT_FRAME_T frame;
                frame.len = len;
                frame.cmd0 = cmd0;
                frame.cmd1 = cmd1;
                if ( len > 0 )
                {
                    memcpy( frame.payload, &parseBuf[idx + 4], len );
                }

                bool isSrsp = ( cmd0 & 0xE0 ) == 0x60;
                if ( isSrsp )
                {
                    pthread_mutex_lock( &s_srspMutex );
                    if ( cmd0 == s_expectedSrspCmd0 && cmd1 == s_expectedSrspCmd1 )
                    {
                        s_srspFrame = frame;
                        s_srspReceived = true;
                        pthread_cond_signal( &s_srspCond );
                    }
                    pthread_mutex_unlock( &s_srspMutex );
                }
                else
                {
                    EventQueue_Push( &g_eventQueue, &frame );
                }

                idx += frameSize;
            }
            else
            {
                idx++; // Corrupt frame or sync loss, scan forward
            }
        }

        if ( idx > 0 )
        {
            if ( idx < parseLen )
            {
                memmove( parseBuf, &parseBuf[idx], parseLen - idx );
                parseLen -= idx;
            }
            else
            {
                parseLen = 0;
            }
        }
    }
    return NULL;
}

// ---------------------------------------------------------------------------
// MT Command API Wrappers
// ---------------------------------------------------------------------------

bool ZNP_SysPing( void )
{
    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x21, 0x01, NULL, 0, &rx, 1000 ) )
    {
        if ( rx.len >= 2 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_SysResetReq( bool hard_ )
{
    uint8_t resetType = hard_ ? 0x00 : 0x01;
    uint8_t payload[1] = { resetType };

    // Clear the event queue BEFORE we write so we don't discard the
    // SYS_RESET_IND that arrives immediately after the chip reboots.
    EventQueue_Clear( &g_eventQueue );

    uint8_t txBuf[6];
    txBuf[0] = 0xFE;
    txBuf[1] = 1;
    txBuf[2] = 0x41;
    txBuf[3] = 0x09;
    txBuf[4] = resetType;
    txBuf[5] = ZNP_CalcFcs( 1, 0x41, 0x09, payload );

    printf( "  TX [SYS_RESET_REQ type=%d (%s)]: ", resetType, hard_ ? "hard" : "soft" );
    for ( int i = 0; i < 6; i++ )
    {
        printf( "%02x", txBuf[i] );
    }
    printf( "\n" );

    // NOTE: Do NOT call tcflush(TCIFLUSH) here. The CC1352P7 sends
    // SYS_RESET_IND within a few milliseconds of receiving the reset command.
    // Flushing the input buffer after the write races with the incoming
    // indication and silently discards it.
    if ( !ZNP_WriteAll( s_serialFd, txBuf, 6 ) )
    {
        return false;
    }
    tcdrain( s_serialFd ); // Ensure all bytes are transmitted before waiting

    printf( "  Waiting for SYS_RESET_IND...\n" );
    // Use 8 seconds; the CC1352P7 can take up to 5 s to erase+reboot after a
    // CLEAR_CONFIG wipe, especially on first boot from a clean chip.
    double deadline = ZNP_GetCurrentTime() + 8.0;
    MT_FRAME_T rx;
    while ( ZNP_GetCurrentTime() < deadline )
    {
        if ( EventQueue_Pop( &g_eventQueue, &rx, 50 ) )
        {
            if ( rx.cmd0 == 0x41 && rx.cmd1 == 0x80 )
            {
                printf( "  -> SYS_RESET_IND: " );
                for ( int i = 0; i < rx.len; i++ )
                {
                    printf( "%02x", rx.payload[i] );
                }
                printf( "\n" );
                return true;
            }
            // Discard non-reset events while waiting for the reset indication
        }
    }
    printf( "  No SYS_RESET_IND received\n" );
    return false;
}

int ZNP_UtilGetDeviceInfo( void )
{
    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x27, 0x00, NULL, 0, &rx, 1000 ) )
    {
        if ( rx.len >= 14 )
        {
            memcpy( g_coordinatorIeee, &rx.payload[1], 8 );
            g_hasCoordinatorIeee = true;

            uint16_t shortAddr = rx.payload[9] | ( rx.payload[10] << 8 );
            uint8_t devType = rx.payload[11];
            uint8_t devState = rx.payload[12];
            uint8_t numAssoc = rx.payload[13];

            printf( "  IEEE: " );
            for ( int i = 7; i >= 0; i-- )
            {
                printf( "%02x", g_coordinatorIeee[i] );
            }
            printf( "\n" );
            printf( "  Short Addr: 0x%04X\n", shortAddr );
            printf( "  Dev Type:   0x%02X\n", devType );
            printf( "  Dev State:  %d  (9=Coordinator)\n", devState );
            printf( "  Associated: %d device(s)\n", numAssoc );
            return devState;
        }
    }
    return -1;
}

int ZNP_ZdoStartupFromApp( uint16_t startDelayMs_ )
{
    uint8_t payload[2];
    payload[0] = startDelayMs_ & 0xFF;
    payload[1] = ( startDelayMs_ >> 8 ) & 0xFF;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x25, 0x40, payload, 2, &rx, 10000 ) )
    {
        if ( rx.len >= 1 )
        {
            uint8_t mode = rx.payload[0];
            const char *label = "unknown";
            if ( mode == 0 )
            {
                label = "Restored from NV";
            }
            else if ( mode == 1 )
            {
                label = "New network";
            }
            else if ( mode == 2 )
            {
                label = "Leave and retry";
            }
            printf( "  StartMode: %d (%s)\n", mode, label );
            return mode;
        }
    }
    return -1;
}

bool ZNP_NvWrite( uint16_t nvId_, const uint8_t *value_, uint8_t len_, uint8_t offset_ )
{
    uint8_t payload[300];
    payload[0] = nvId_ & 0xFF;
    payload[1] = ( nvId_ >> 8 ) & 0xFF;
    payload[2] = offset_;
    payload[3] = len_;
    memcpy( &payload[4], value_, len_ );

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x21, 0x09, payload, 4 + len_, &rx, 1000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_NvRead( uint16_t nvId_, uint8_t offset_, uint8_t *rxBuf_, uint8_t *rxLen_ )
{
    uint8_t payload[3];
    payload[0] = nvId_ & 0xFF;
    payload[1] = ( nvId_ >> 8 ) & 0xFF;
    payload[2] = offset_;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x21, 0x08, payload, 3, &rx, 1000 ) )
    {
        if ( rx.len >= 2 && rx.payload[0] == 0 )
        {
            uint8_t len = rx.payload[1];
            if ( rxBuf_ != NULL && rxLen_ != NULL )
            {
                *rxLen_ = len;
                memcpy( rxBuf_, &rx.payload[2], len );
            }
            return true;
        }
    }
    return false;
}

bool ZNP_WriteConfiguration( uint8_t cfgId_, const uint8_t *value_, uint8_t len_ )
{
    bool ok = ZNP_NvWrite( cfgId_, value_, len_, 0 );

    uint8_t readback[64] = { 0 };
    uint8_t readLen = 0;
    bool readOk = ZNP_NvRead( cfgId_, 0, readback, &readLen );

    bool match = readOk && ( readLen == len_ ) && ( memcmp( readback, value_, len_ ) == 0 );

    printf( "  NV_WRITE id=0x%02X value=", cfgId_ );
    for ( int i = 0; i < len_; i++ )
    {
        printf( "%02X", value_[i] );
    }
    printf( " write=%s ", ok ? "✅" : "❌" );

    if ( readOk )
    {
        printf( "readback=" );
        for ( int i = 0; i < readLen; i++ )
        {
            printf( "%02X", readback[i] );
        }
        printf( " %s\n", match ? "✅" : "❌" );
    }
    else
    {
        printf( "readback=None ❌\n" );
    }

    if ( cfgId_ == ZCD_NV_PRECFGKEY && ok )
    {
        return true;
    }

    return ok && match;
}

bool ZNP_FactoryNew( void )
{
    // -----------------------------------------------------------------------
    // Phase 1: Set the NV-wipe flag and reset so the chip actually clears NV.
    //
    // IMPORTANT ORDERING: We must NOT clear STARTUP_OPTION back to 0 until
    // after the reset that triggers the wipe has been confirmed. If we clear
    // it early (before the reset fires), the wipe flag is gone and the chip
    // boots with stale NV data, causing the state-8->0 loop.
    // -----------------------------------------------------------------------
    printf( "[FN-1] Set STARTUP_OPTION = CLEAR_CONFIG | CLEAR_STATE...\n" );
    uint8_t opt = STARTOPT_CLEAR_CONFIG | STARTOPT_CLEAR_STATE;
    ZNP_WriteConfiguration( ZCD_NV_STARTUP_OPTION, &opt, 1 );

    printf( "[FN-2] SYS_RESET_REQ to trigger NV wipe...\n" );
    bool resetOk = ZNP_SysResetReq( false );
    if ( !resetOk )
    {
        printf( "  Soft reset unconfirmed - escalating to hard reset...\n" );
        resetOk = ZNP_SysResetReq( true );
    }
    if ( resetOk )
    {
        // Give the chip time to finish erasing NV before we talk to it again
        printf( "  Reset confirmed. Waiting 2s for NV erase to complete...\n" );
        sleep( 2 );
    }
    else
    {
        printf( "  ⚠️  Both resets unconfirmed. Proceeding anyway after 3s delay...\n" );
        sleep( 3 );
    }

    // -----------------------------------------------------------------------
    // Phase 2: Write all configuration items onto the now-clean NV.
    // -----------------------------------------------------------------------
    printf( "[FN-3] Re-set STARTUP_OPTION = 0 (don't wipe on subsequent boots)\n" );
    opt = 0;
    ZNP_WriteConfiguration( ZCD_NV_STARTUP_OPTION, &opt, 1 );

    printf( "[FN-4] Logical type = Coordinator (0x00)\n" );
    uint8_t type = 0x00;
    ZNP_WriteConfiguration( ZCD_NV_LOGICAL_TYPE, &type, 1 );

    printf( "[FN-5] PAN ID = 0x%04X\n", NETWORK_PAN_ID );
    uint8_t pan[2];
    pan[0] = NETWORK_PAN_ID & 0xFF;
    pan[1] = ( NETWORK_PAN_ID >> 8 ) & 0xFF;
    ZNP_WriteConfiguration( ZCD_NV_PANID, pan, 2 );

    printf( "[FN-6] Channel list = ch %d only (0x%08X)\n", NETWORK_CHANNEL, 1 << NETWORK_CHANNEL );
    uint32_t chan = 1 << NETWORK_CHANNEL;
    uint8_t chanBytes[4];
    chanBytes[0] = chan & 0xFF;
    chanBytes[1] = ( chan >> 8 ) & 0xFF;
    chanBytes[2] = ( chan >> 16 ) & 0xFF;
    chanBytes[3] = ( chan >> 24 ) & 0xFF;
    ZNP_WriteConfiguration( ZCD_NV_CHANLIST, chanBytes, 4 );

    printf( "[FN-7] Pre-configured key = %s\n", NETWORK_TC_LINK_KEY_HEX );
    uint8_t key[16];
    for ( int i = 0; i < 16; i++ )
    {
        sscanf( &NETWORK_TC_LINK_KEY_HEX[i * 2], "%2hhx", &key[i] );
    }
    ZNP_WriteConfiguration( ZCD_NV_PRECFGKEY, key, 16 );

    printf( "[FN-8] PRECFGKEYS_ENABLE = 0 (TC link key joining)\n" );
    uint8_t preCfg = 0;
    ZNP_WriteConfiguration( ZCD_NV_PRECFGKEYS_ENABLE, &preCfg, 1 );

    printf( "[FN-9] ZDO_DIRECT_CB = 1 (so we receive STATE_CHANGE_IND)\n" );
    uint8_t cb = 1;
    ZNP_WriteConfiguration( ZCD_NV_ZDO_DIRECT_CB, &cb, 1 );

    // -----------------------------------------------------------------------
    // Phase 3: Final reset to load all the freshly-written configuration.
    // -----------------------------------------------------------------------
    printf( "[FN-10] SYS_RESET_REQ to apply written configurations...\n" );
    resetOk = ZNP_SysResetReq( false );
    if ( !resetOk )
    {
        printf( "  Soft reset unconfirmed - escalating to hard reset...\n" );
        resetOk = ZNP_SysResetReq( true );
    }
    if ( resetOk )
    {
        printf( "  Reset confirmed. Waiting 2s for startup...\n" );
        sleep( 2 );
    }
    else
    {
        printf( "  ⚠️  Final reset unconfirmed. Proceeding after 3s delay...\n" );
        sleep( 3 );
    }
    return true;
}

bool ZNP_BdbSetTcRequireKeyExchange( bool require_ )
{
    printf( "  [BDB] Set TC require key exchange = %s\n", require_ ? "TRUE" : "FALSE" );
    uint8_t payload[1];
    payload[0] = require_ ? 0x01 : 0x00;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x2F, 0x09, payload, 1, &rx, 3000 ) )
    {
        uint8_t status = rx.len >= 1 ? rx.payload[0] : 0xFF;
        printf( "    status=%d %s\n", status, status == 0 ? "✅" : "❌" );
        return status == 0;
    }
    printf( "    ❌ no response\n" );
    return false;
}

bool ZNP_PermitJoin( uint8_t duration_ )
{
    bool ok = false;
    MT_FRAME_T rx;

    printf( "  [PJ-1] ZDO_MGMT_PERMIT_JOIN_REQ → coordinator (0x0000), %ds\n", duration_ );
    uint8_t payload[5];
    payload[0] = 0x02;
    payload[1] = 0x00;
    payload[2] = 0x00;
    payload[3] = duration_;
    payload[4] = 0x01;
    if ( ZNP_Sreq( 0x25, 0x36, payload, 5, &rx, 3000 ) )
    {
        uint8_t status = rx.len >= 1 ? rx.payload[0] : 0xFF;
        printf( "    status=%d %s\n", status, status == 0 ? "✅" : "❌" );
        ok = ok || ( status == 0 );
    }

    printf( "  [PJ-2] ZDO_MGMT_PERMIT_JOIN_REQ → broadcast (0xFFFC), %ds\n", duration_ );
    payload[0] = 0x0F;
    payload[1] = 0xFC;
    payload[2] = 0xFF;
    payload[3] = duration_;
    payload[4] = 0x01;
    if ( ZNP_Sreq( 0x25, 0x36, payload, 5, &rx, 3000 ) )
    {
        uint8_t status = rx.len >= 1 ? rx.payload[0] : 0xFF;
        printf( "    status=%d %s\n", status, status == 0 ? "✅" : "❌" );
        ok = ok || ( status == 0 );
    }

    printf( "  [PJ-3] ZB_PERMIT_JOINING_REQ, %ds\n", duration_ );
    uint8_t zbPay[3];
    zbPay[0] = 0xFC;
    zbPay[1] = 0xFF;
    zbPay[2] = duration_;
    if ( ZNP_Sreq( 0x26, 0x08, zbPay, 3, &rx, 3000 ) )
    {
        uint8_t status = rx.len >= 1 ? rx.payload[0] : 0xFF;
        printf( "    status=%d %s\n", status, status == 0 ? "✅" : "❌" );
    }

    printf( "  [PJ-4] BDB_START_COMMISSIONING (Network Steering)\n" );
    uint8_t bdbPay[1];
    bdbPay[0] = 0x02; // Network Steering
    if ( ZNP_Sreq( 0x2F, 0x05, bdbPay, 1, &rx, 3000 ) )
    {
        uint8_t status = rx.len >= 1 ? rx.payload[0] : 0xFF;
        printf( "    status=%d %s\n", status, status == 0 ? "✅" : "❌" );
        ok = ok || ( status == 0 );
    }

    return ok;
}

bool ZNP_AfRegister( uint8_t endpoint_, uint16_t profileId_, uint16_t deviceId_,
                     uint8_t deviceVersion_, uint8_t latency_,
                     uint8_t numInClusters_, const uint16_t *inClusters_,
                     uint8_t numOutClusters_, const uint16_t *outClusters_ )
{
    uint8_t payload[250];
    int idx = 0;

    payload[idx++] = endpoint_;
    payload[idx++] = profileId_ & 0xFF;
    payload[idx++] = ( profileId_ >> 8 ) & 0xFF;
    payload[idx++] = deviceId_ & 0xFF;
    payload[idx++] = ( deviceId_ >> 8 ) & 0xFF;
    payload[idx++] = deviceVersion_;
    payload[idx++] = latency_;

    payload[idx++] = numInClusters_;
    for ( int i = 0; i < numInClusters_; i++ )
    {
        payload[idx++] = inClusters_[i] & 0xFF;
        payload[idx++] = ( inClusters_[i] >> 8 ) & 0xFF;
    }

    payload[idx++] = numOutClusters_;
    for ( int i = 0; i < numOutClusters_; i++ )
    {
        payload[idx++] = outClusters_[i] & 0xFF;
        payload[idx++] = ( outClusters_[i] >> 8 ) & 0xFF;
    }

    printf( "Registering endpoint %d...\n", endpoint_ );
    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x24, 0x00, payload, idx, &rx, 3000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            printf( "  Endpoint %d registered successfully!\n", endpoint_ );
            return true;
        }
        else
        {
            printf( "  Failed to register endpoint %d. Status=%d\n",
                    endpoint_, rx.len >= 1 ? rx.payload[0] : -1 );
        }
    }
    else
    {
        printf( "  Failed to register endpoint %d. Status=TIMEOUT\n", endpoint_ );
    }
    return false;
}

bool ZNP_ZdoMsgCbRegister( uint16_t clusterId_ )
{
    printf( "Registering ZDO callback for cluster 0x%04X...\n", clusterId_ );
    uint8_t payload[2];
    payload[0] = clusterId_ & 0xFF;
    payload[1] = ( clusterId_ >> 8 ) & 0xFF;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x25, 0x3E, payload, 2, &rx, 1000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_AfDataRequestExt( uint8_t dstAddrMode_, uint64_t dstAddr_, uint8_t dstEndpoint_,
                           uint16_t panId_, uint8_t srcEndpoint_, uint16_t clusterId_,
                           uint8_t transId_, uint8_t options_, uint8_t radius_,
                           const uint8_t *data_, uint16_t dataLen_ )
{
    uint8_t payload[300];
    int idx = 0;

    payload[idx++] = dstAddrMode_;
    if ( dstAddrMode_ == 3 )
    {
        for ( int i = 0; i < 8; i++ )
        {
            payload[idx++] = ( dstAddr_ >> ( i * 8 ) ) & 0xFF;
        }
    }
    else
    {
        payload[idx++] = dstAddr_ & 0xFF;
        payload[idx++] = ( dstAddr_ >> 8 ) & 0xFF;
        memset( &payload[idx], 0, 6 );
        idx += 6;
    }

    payload[idx++] = dstEndpoint_;
    payload[idx++] = panId_ & 0xFF;
    payload[idx++] = ( panId_ >> 8 ) & 0xFF;
    payload[idx++] = srcEndpoint_;
    payload[idx++] = clusterId_ & 0xFF;
    payload[idx++] = ( clusterId_ >> 8 ) & 0xFF;
    payload[idx++] = transId_;
    payload[idx++] = options_;
    payload[idx++] = radius_;
    payload[idx++] = dataLen_ & 0xFF;
    payload[idx++] = ( dataLen_ >> 8 ) & 0xFF;
    memcpy( &payload[idx], data_, dataLen_ );
    idx += dataLen_;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x24, 0x02, payload, idx, &rx, 3000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_ZdoMatchDescReq( uint16_t shortAddr_, uint16_t profileId_,
                          uint8_t numInClusters_, const uint16_t *inClusters_,
                          uint8_t numOutClusters_, const uint16_t *outClusters_ )
{
    uint8_t payload[200];
    int idx = 0;

    payload[idx++] = shortAddr_ & 0xFF;
    payload[idx++] = ( shortAddr_ >> 8 ) & 0xFF;
    payload[idx++] = shortAddr_ & 0xFF;
    payload[idx++] = ( shortAddr_ >> 8 ) & 0xFF;
    payload[idx++] = profileId_ & 0xFF;
    payload[idx++] = ( profileId_ >> 8 ) & 0xFF;

    payload[idx++] = numInClusters_;
    for ( int i = 0; i < numInClusters_; i++ )
    {
        payload[idx++] = inClusters_[i] & 0xFF;
        payload[idx++] = ( inClusters_[i] >> 8 ) & 0xFF;
    }

    payload[idx++] = numOutClusters_;
    for ( int i = 0; i < numOutClusters_; i++ )
    {
        payload[idx++] = outClusters_[i] & 0xFF;
        payload[idx++] = ( outClusters_[i] >> 8 ) & 0xFF;
    }

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x25, 0x06, payload, idx, &rx, 3000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_ZdoActiveEpReq( uint16_t shortAddr_ )
{
    printf( " Sending Active EP Request to 0x%04X...\n", shortAddr_ );
    uint8_t payload[4];
    payload[0] = shortAddr_ & 0xFF;
    payload[1] = ( shortAddr_ >> 8 ) & 0xFF;
    payload[2] = shortAddr_ & 0xFF;
    payload[3] = ( shortAddr_ >> 8 ) & 0xFF;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x25, 0x05, payload, 4, &rx, 3000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_QuerySimpleDesc( uint16_t shortAddr_, uint8_t endpoint_ )
{
    printf( " Sending Simple Desc Request to 0x%04X ep 0x%02X...\n", shortAddr_, endpoint_ );
    uint8_t payload[5];
    payload[0] = shortAddr_ & 0xFF;
    payload[1] = ( shortAddr_ >> 8 ) & 0xFF;
    payload[2] = shortAddr_ & 0xFF;
    payload[3] = ( shortAddr_ >> 8 ) & 0xFF;
    payload[4] = endpoint_;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x25, 0x04, payload, 5, &rx, 3000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_ZdoBindReq( uint16_t buttonShortAddr_, const uint8_t *buttonIeee_, uint8_t srcEndpoint_,
                     uint16_t clusterId_, const uint8_t *coordIeee_, uint8_t coordEndpoint_ )
{
    printf( "Sending ZDO Bind Request to button 0x%04X...\n", buttonShortAddr_ );
    uint8_t payload[25];
    int idx = 0;

    payload[idx++] = buttonShortAddr_ & 0xFF;
    payload[idx++] = ( buttonShortAddr_ >> 8 ) & 0xFF;

    memcpy( &payload[idx], buttonIeee_, 8 );
    idx += 8;

    payload[idx++] = srcEndpoint_;
    payload[idx++] = clusterId_ & 0xFF;
    payload[idx++] = ( clusterId_ >> 8 ) & 0xFF;

    payload[idx++] = 3; // DstAddrMode = 64-bit IEEE

    memcpy( &payload[idx], coordIeee_, 8 );
    idx += 8;

    payload[idx++] = coordEndpoint_;

    MT_FRAME_T rx;
    if ( ZNP_Sreq( 0x25, 0x21, payload, idx, &rx, 3000 ) )
    {
        if ( rx.len >= 1 && rx.payload[0] == 0 )
        {
            return true;
        }
    }
    return false;
}

bool ZNP_WriteCieAddress( uint16_t buttonShortAddr_, uint8_t buttonEndpoint_, uint8_t transId_ )
{
    printf( "Writing CIE Address to button 0x%04X...\n", buttonShortAddr_ );

    uint8_t zclPayload[11];
    zclPayload[0] = 0x10;
    zclPayload[1] = 0x00;
    zclPayload[2] = 0xF0;
    memcpy( &zclPayload[3], g_coordinatorIeee, 8 );

    uint8_t zclFrame[14];
    zclFrame[0] = 0x00;
    zclFrame[1] = transId_;
    zclFrame[2] = 0x02;
    memcpy( &zclFrame[3], zclPayload, 11 );

    return ZNP_AfDataRequestExt( 2, buttonShortAddr_, buttonEndpoint_, 0, 8, 0x0500,
                                 transId_, 0, 30, zclFrame, 14 );
}

bool ZNP_SendButtonActivation( uint16_t buttonShortAddr_, uint8_t buttonEndpoint_, uint8_t transId_ )
{
    // NOTE: This function is intentionally left as a no-op.
    // The SBTZB-110 (Smart Button) does NOT support the IAS Zone / Panic
    // cluster. Attribute 0x8000 on cluster 0x000F is only valid on the
    // PBTZB-110 (Panic Button) — a different hardware model.
    // Sending this write to SBTZB-110 always returns 0x8D INVALID_DATA_TYPE.
    (void)buttonShortAddr_;
    (void)buttonEndpoint_;
    (void)transId_;
    return true;
}

bool ZNP_SendZoneEnrollResponse( uint16_t buttonShortAddr_, uint8_t buttonEndpoint_,
                                 uint8_t transId_, uint8_t zoneId_ )
{
    printf( "Sending Zone Enroll Response to button 0x%04X...\n", buttonShortAddr_ );

    uint8_t zclFrame[5];
    zclFrame[0] = 0x11;
    zclFrame[1] = transId_;
    zclFrame[2] = 0x00;
    zclFrame[3] = 0x00;
    zclFrame[4] = zoneId_;

    return ZNP_AfDataRequestExt( 2, buttonShortAddr_, buttonEndpoint_, 0, 8, 0x0500,
                                 transId_, 0, 30, zclFrame, 5 );
}

bool ZNP_SendSirenWarning( uint16_t sirenShortAddr_, uint8_t sirenEndpoint_, uint8_t transId_,
                           uint8_t warnMode_, uint8_t volume_, uint16_t duration_ )
{
    const char *modeStr = ( warnMode_ != 0 ) ? "START" : "STOP";
    printf( "Sending Siren %s to siren 0x%04X ep=0x%02X...\n", modeStr, sirenShortAddr_, sirenEndpoint_ );

    uint8_t modeLevel = ( ( warnMode_ & 0x0F ) << 4 ) | (volume_ & 0x03);
    uint8_t zclFrame[8];
    zclFrame[0] = 0x11;
    zclFrame[1] = transId_;
    zclFrame[2] = 0x00;
    zclFrame[3] = modeLevel;
    zclFrame[4] = duration_ & 0xFF;
    zclFrame[5] = ( duration_ >> 8 ) & 0xFF;
    zclFrame[6] = 0x00;
    zclFrame[7] = 0x00;

    return ZNP_AfDataRequestExt( 2, sirenShortAddr_, sirenEndpoint_, 1, 8, 0x0502,
                                 transId_, 0, 30, zclFrame, 8 );
}
bool ZNP_SendSirenSquawk( uint16_t sirenShortAddr_, uint8_t sirenEndpoint_, uint8_t transId_,
                          uint8_t squawkMode_, uint8_t volume_ )
{
    printf( "Sending Siren SQUAWK to siren 0x%04X ep=0x%02X...\n", sirenShortAddr_, sirenEndpoint_ );

    // Bits 4-7: Squawk Mode (0=System is armed)
    // Bit 3: Strobe (0)
    // Bits 0-1: Squawk Level
    uint8_t squawkInfo = ( ( squawkMode_ & 0x0F ) << 4 ) | (volume_ & 0x03); 
    
    uint8_t zclFrame[4];
    zclFrame[0] = 0x11; // 0x11 = Cluster specific, Client to Server, Default Response DISABLED
    zclFrame[1] = transId_;
    zclFrame[2] = 0x01; // Command: Squawk
    zclFrame[3] = squawkInfo;

    return ZNP_AfDataRequestExt( 2, sirenShortAddr_, sirenEndpoint_, 1, 4, 0x0502,
                                 transId_, 0, 30, zclFrame, 4 );
}




bool ZNP_SendDefaultResponse(uint16_t shortAddr_, uint8_t endpoint_, uint16_t clusterId_,
                             uint8_t transId_, uint8_t cmdId_, uint8_t status_)
{
    printf("Sending Default Response to 0x%04X ep=0x%02X cluster=0x%04X cmd=0x%02X status=0x%02X...\n",
           shortAddr_, endpoint_, clusterId_, cmdId_, status_);

    uint8_t zclFrame[5];
    zclFrame[0] = 0x10;
    zclFrame[1] = transId_;
    zclFrame[2] = 0x0B;
    zclFrame[3] = cmdId_;
    zclFrame[4] = status_;

    return ZNP_AfDataRequestExt(2, shortAddr_, endpoint_, 0, 8, clusterId_,
                                transId_, 0, 30, zclFrame, 5);
}
