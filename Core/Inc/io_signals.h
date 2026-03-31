#ifndef IO_SIGNALS_H
#define IO_SIGNALS_H

#include "control.h"

/* Functional bridge between the PCB signal names and the current CubeMX labels.
 *
 * Production source-of-truth:
 * - APPS1 / APPS2 / brake / start come from local ADC/GPIO.
 * - CAN bus 3 must not overwrite driver inputs.
 */
#define IO_DRIVER_INPUTS_SOURCE_LOCAL_IO  1u
#define IO_DRIVER_INPUTS_SOURCE           IO_DRIVER_INPUTS_SOURCE_LOCAL_IO

void IoSignals_Init(void);
void IoSignals_InputStep(void);
void IoSignals_ApplyOutputs(const control_out_t *out);

#endif /* IO_SIGNALS_H */
