#ifndef INC_PWM_H_
#define INC_PWM_H_

#include "main.h"
/*	Функция запускающая ШИМ		*/
void PWM_start(float freq, float duty);
/*	Функция остановки ШИМ	*/
void PWM_stop();

#endif /* INC_PWM_H_ */
