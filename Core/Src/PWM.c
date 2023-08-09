#include "PWM.h"

void PWM_start(float freq, float duty)
{
	/*	PSC = 1  всегда. Таймер 5 на 32 бита для того чтобы можно было делать сверхнизкие частоты	*/
	uint32_t tim4_counter_period = 64000 / freq;	
	uint32_t tim4_ccr_val = (tim4_counter_period * duty) / 100; 
	TIM5->PSC = 0;
	TIM5->CNT = 0;
	TIM5->ARR = tim4_counter_period;
	TIM5->CCR1 = tim4_ccr_val;
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
}
void PWM_stop()
{
	TIM5->CNT = 0;
	HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
}
