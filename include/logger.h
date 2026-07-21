#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include <time.h>

// Log Levels
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_WARNING = 2,
    LOG_LEVEL_ERROR = 3
} LogLevel;

extern LogLevel g_logLevel;

// Internal timestamp helper
void Logger_PrintTimestamp(void);

// Logging Macros
#define LOG_RAW(...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_DEBUG) { \
            printf(__VA_ARGS__); \
        } \
    } while(0)

#define LOG_DEBUG(...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_DEBUG) { \
            Logger_PrintTimestamp(); \
            printf("[DEBUG] "); \
            printf(__VA_ARGS__); \
        } \
    } while(0)

#define LOG_INFO(...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_INFO) { \
            Logger_PrintTimestamp(); \
            printf("[INFO]  "); \
            printf(__VA_ARGS__); \
        } \
    } while(0)

#define LOG_WARNING(...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_WARNING) { \
            Logger_PrintTimestamp(); \
            printf("[WARN]  "); \
            printf(__VA_ARGS__); \
        } \
    } while(0)

#define LOG_ERROR(...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_ERROR) { \
            Logger_PrintTimestamp(); \
            printf("[ERROR] "); \
            printf(__VA_ARGS__); \
        } \
    } while(0)

#define LOG_EVENT(MODULE, ADDR, ...) \
    do { \
        if (g_logLevel <= LOG_LEVEL_INFO) { \
            printf("\n"); \
            Logger_PrintTimestamp(); \
            printf("[EVENT] [%-10s] [0x%04X] ", MODULE, ADDR); \
            printf(__VA_ARGS__); \
        } \
    } while(0)

#endif // LOGGER_H
