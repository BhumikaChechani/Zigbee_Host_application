#include "logger.h"

LogLevel g_logLevel = LOG_LEVEL_INFO;

void Logger_PrintTimestamp(void) {
    time_t now;
    time(&now);
    struct tm *local = localtime(&now);
    printf("[%02d:%02d:%02d] ", local->tm_hour, local->tm_min, local->tm_sec);
}
