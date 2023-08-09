#ifndef __DAC_SOFT_H
#define __DAC_SOFT_H

#include "main.h"
#include "cmsis_os.h"

/*	Типы сигналов данного ЦАПа	*/
typedef enum {
	NONE = 0,
	TRIANGLE,
	SIN,
	SAW,
	REVERSE_SAW
} signal_t;

/*	Функция старта программного ЦАПа. Принимает аргументами тип сигнала и его частоту в кГц	*/
void DAC_start(signal_t signal, float freq);
/*	Функция остановки программного ЦАПа	*/
void DAC_stop();



#endif
