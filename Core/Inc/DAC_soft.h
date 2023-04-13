#ifndef __DAC_SOFT_H
#define __DAC_SOFT_H

#include "main.h"
#include "cmsis_os.h"

#define	DAC_STEP			5

typedef enum {
	TRIANGLE = 0,
	SIN,
	SAW,
	REVERSE_SAW
} signal_t;

void DAC_init(signal_t signal, float freq);
void DAC_stop();
void DAC_writeTriangle();
void DAC_writeSin();
void DAC_writeSaw();
void DAC_writeReverseSaw();

extern xSemaphoreHandle xDacTickSemaphore;

#endif
