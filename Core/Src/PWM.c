#include "PWM.h"

void PWM_start(float freq, float duty)
{
	/*	PSC = 1  всегда, значит минимальная частота Ш�?М около 1 кГц	*/
	uint16_t tim4_counter_period = 64000 / freq;	// Расчёт в kHz, при PSC = 1
	uint16_t tim4_ccr_val = (tim4_counter_period * duty) / 100; // Обычная пропорция ARR - 100% а CCR - duty, откуда нашли CCR
	TIM4->PSC = 0;
	TIM4->CNT = 0;
	TIM4->ARR = tim4_counter_period;
	TIM4->CCR1 = tim4_ccr_val;
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
}
void PWM_stop()
{
	TIM4->CNT = 0;
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
}
