#ifndef DIAG_H
#define DIAG_H

#include <stdint.h>
#include "cmsis_os2.h"

void Diag_Report(osMessageQueueId_t rxQ, osMessageQueueId_t txQ);

/* Formatted logging (printf-style). Default weak no-op. */
void Diag_Log(const char *fmt, ...);

#endif /* DIAG_H */
