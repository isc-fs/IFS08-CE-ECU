#ifndef IO_SIGNALS_H
#define IO_SIGNALS_H

#include "control.h"

/* Functional bridge between the PCB signal names and the current CubeMX labels. */

void IoSignals_Init(void);
void IoSignals_InputStep(void);
void IoSignals_ApplyOutputs(const control_out_t *out);

#endif /* IO_SIGNALS_H */
