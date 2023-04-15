#include "DAC_soft.h"

xSemaphoreHandle xDacTickSemaphore;

uint8_t sin_tab[256] = {
127,130,133,136,139,142,145,148,151,154,157,160,164,166,169,172,175,178,181,184,187,189,192,195,
197,200,202,205,207,210,212,214,217,219,221,223,225,227,229,231,232,234,236,237,239,240,242,243,
244,245,246,247,248,249, 250,251,251,252,252,253,253,253,253,253,254,253,253,253,253,252,252,251,
251,250,249,249,248,247,246,245,243,242,241,239,238,236,235,233,231,230, 228,226,224,222,220,218,
215,213,211,209,206,204,201,199,196,193,191,188,185,182,180,177,174,171,168,165,162,159,156,153,
150,147,144,141,137,134,131,128,125,122,119,116,112,109,106,103,100,97,94,91,88,85,82,79,76, 73,
71,68,65,62,60,57,54,52,49,47,44,42,40,38,35,33,31,29,27,25,23,22,20,18,17,15,14,12,11,10,8,7,6,
5,4,4,3,2,2,1,1,0,0,0,0,0,0,0,0,0,0,1,1,2,2,3,4,5,6,7,8,9,10,11,13,14,16,17,19,21,22,24,26,28,30,
32,34,36,39,41,43,46,48,51,53,56,58,61,64,66,69,72,75,78,81,84,87,89,93,96,99,102,105,108,111,114,
117,120,123,127};

static float one_dac_tick;
static uint16_t tim3_counter_period;

void DAC_stop()
{
	TIM3->CNT = 0;
	GPIOB->ODR = 0;
	HAL_TIM_Base_Stop_IT(&htim3);
}
void DAC_init(signal_t signal, float freq)
{
	if( signal == SAW || signal == REVERSE_SAW ) // Для обычной и обратной пилы настройки одинаковые
	{
		one_dac_tick = (1000 / freq) / (51);
	}
	else if(signal == TRIANGLE)
	{
		/*	Частота таймера базовая будет 64 Мгц. Предделитель допустим всегда 1. Шаг ЦАПа 5
			51*2 = 102 ступенек если выбрать шаг 5. Вычислили тик ЦАПа в мкс чтобы настроить на него таймер. 	*/
		one_dac_tick = (1000 / freq) / (102);
	}
	else if( signal == SIN )
	{
		one_dac_tick = (1000 / freq) / (256);
	}
	tim3_counter_period = one_dac_tick / 0.015625;
	TIM3->PSC = 0;
	TIM3->ARR = tim3_counter_period;
	TIM3->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim3);		// Запустили таймер
	TIM3->SR &= ~(TIM_SR_UIF);			// Сбросим флаг
}

void DAC_writeSin()
{
	int j = 0;
	while( j < sizeof(sin_tab)/sizeof(uint8_t) )
	{
		if( (TIM3->SR & TIM_SR_UIF) )
		{
			TIM3->SR &= ~(TIM_SR_UIF);
			GPIOB->ODR = sin_tab[j];
			j++;
		}
	}
}

void DAC_writeTriangle()	// freq in kHz
{
	int j = 0;
	while (j < 255)
	{
		if( (TIM3->SR & TIM_SR_UIF) )
		{
			TIM3->SR &= ~(TIM_SR_UIF);
			GPIOB->ODR = j;
			j += 5;
		}
	}
	while(j > 0)
	{
		if( (TIM3->SR & TIM_SR_UIF) )
		{
			TIM3->SR &= ~(TIM_SR_UIF);
			GPIOB->ODR = j;
			j -= 5;
		}
	}
}
void DAC_writeSaw()		// in kHz
{
	/*	Частота таймера базовая будет 64 Мгц. Предделитель допустим всегда 1. Шаг ЦАПа 5. 	*/
	int j = 0;
	while (j <= 255)
	{
		if( (TIM3->SR & TIM_SR_UIF) )
		{
			GPIOB->ODR = j;
			TIM3->SR &= ~(TIM_SR_UIF);
			j += 5;
		}
	}
}
void DAC_writeReverseSaw()		// in kHz
{
	/*	Частота таймера базовая будет 64 Мгц. Предделитель допустим всегда 1. Шаг ЦАПа 5. 	*/
	int j = 255;
	while (j > 0)
	{
		if( (TIM3->SR & TIM_SR_UIF) )
		{
			GPIOB->ODR = j;
			TIM3->SR &= ~(TIM_SR_UIF);
			j -= 5;
		}
	}
}

