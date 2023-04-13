#ifndef INC_MAX7219_H_
#define INC_MAX7219_H_

#include "main.h"
#include "string.h"
#include "stdio.h"


#define POINT		0x80
//----------------------------------------------------Адреса регистров------------------------------------------------------
#define MAX7219_DECODE_MODE_REG				0x09
#define MAX7219_INTENSITY_REG				0x0A
#define MAX7219_SCAN_LIMIT_REG				0x0B
#define MAX7219_MODE_REG					0x0C	// (Shutdown register)
#define MAX7219_DIGIT_0_REG					0x01
#define MAX7219_DIGIT_1_REG					0x02
#define MAX7219_DIGIT_2_REG					0x03
#define MAX7219_DIGIT_3_REG					0x04
#define MAX7219_DIGIT_4_REG					0x05
#define MAX7219_DIGIT_5_REG					0x06
#define MAX7219_DIGIT_6_REG					0x07
#define MAX7219_DIGIT_7_REG					0x08
//----------------------------------------------------Константы для пользователя--------------------------------------------
#define MAX7219_SEGMENT_0 					MAX7219_DIGIT_0_REG
#define MAX7219_SEGMENT_1 					MAX7219_DIGIT_1_REG
#define MAX7219_SEGMENT_2 					MAX7219_DIGIT_2_REG
#define MAX7219_SEGMENT_3 					MAX7219_DIGIT_3_REG
#define MAX7219_SEGMENT_4 					MAX7219_DIGIT_4_REG
#define MAX7219_SEGMENT_5 					MAX7219_DIGIT_5_REG
#define MAX7219_SEGMENT_6 					MAX7219_DIGIT_6_REG
#define MAX7219_SEGMENT_7 					MAX7219_DIGIT_7_REG

#define MAX7219_SEGMENTS_AMOUNT 			MAX7219_SEGMENT_7 			// Всего 8 сегментов
//------------------------------------------------------Режимы декодирования-------------------------------------------------
#define MAX7219_DECODE_FOR_ALL_DIGITS		0xFF		// Декодирование на всех сегментах
#define MAX7219_DECODE_FOR_DIGIT_0			0x01		// Декодирование 0-ого сегмента(самого правого)
#define MAX7219_DECODE_FOR_DIGITS_3_TO_0	0x0F		// Декодирование сегментов 0-3
#define MAX7219_NO_DECODE					0
//--------------------------------------------------------Режим сна или нет--------------------------------------------------
#define MAX7219_NORMAL_MODE					0x01		// НЕ спящий режим
//--------------------------------------------------Символы для режима без кодирования---------------------------------------
#define _MINUS	0x01
#define _0 		0x7E		// Здесь свои символы, отличные от нормальных для 7-сегментного инидкатора
#define _1 		0x30
#define _2 		0x6D
#define _3 		0x79
#define _4 		0x33
#define _5 		0x5B
#define _6 		0x5F
#define _7 		0x70
#define _8 		0x7f
#define _9 		0x7B
#define _A 		0x77
#define _B 		0x1F
#define _C 		0x4E
#define _D 		0x3D
#define _E 		0x4F
#define _F 		0x47
#define _G 		0x5E
#define _H 		0x37
#define _I		0x30
#define _J 		0x3C
#define _K		0x2F
#define _L 		0x0E
#define _M		0x55
#define _N 		0x15
#define _O 		0x1D
#define _P 		0x67
#define _Q		0x73
#define _R		0x05
#define _S 		0x5B
#define _T		0x0F
#define _U 		0x3E
#define _V		0x1C
#define _W		0x5C
#define _X		0x49
#define _Y 		0x3B
#define _Z		0x6D
#define _SPACE	0x00
//---------------------------------------------------------------------------------------------------------------------------
typedef enum {
	MAX7219_INTENSITY_3_OF_32 = 0x01,		// Интенсивность свечения
	MAX7219_INTENSITY_5_OF_32,
	MAX7219_INTENSITY_7_OF_32,
	MAX7219_INTENSITY_9_OF_32,
	MAX7219_INTENSITY_11_OF_32,
	MAX7219_INTENSITY_13_OF_32,
	MAX7219_INTENSITY_15_OF_32,
	MAX7219_INTENSITY_17_OF_32,
	MAX7219_INTENSITY_19_OF_32,
	MAX7219_INTENSITY_21_OF_32,
	MAX7219_INTENSITY_23_OF_32,
	MAX7219_INTENSITY_25_OF_32,
	MAX7219_INTENSITY_27_OF_32,
	MAX7219_INTENSITY_29_OF_32,
	MAX7219_INTENSITY_31_OF_32
} max7219_intensity_t;

typedef enum {
	MAX7219_DISPLAY_0,
	MAX7219_DISPLAY_0_TO_1,				// Сколько сегментов включаем
	MAX7219_DISPLAY_0_TO_2,
	MAX7219_DISPLAY_0_TO_3,
	MAX7219_DISPLAY_0_TO_4,
	MAX7219_DISPLAY_0_TO_5,
	MAX7219_DISPLAY_0_TO_6,
	MAX7219_DISPLAY_0_TO_7,		// (all digits)
} display_segments_quantity_t;

typedef struct {						// Структуру для инициализации и состояния
	SPI_HandleTypeDef *SPI_Handle;
	uint32_t CS_PIN;
	uint32_t CS_PORT;
	uint8_t decode_mode;
	max7219_intensity_t indicator_intensity;
	display_segments_quantity_t digits_quantity;
} max7219_init_t;


HAL_StatusTypeDef MAX7219_init(max7219_init_t *cfg);
uint8_t MAX7219_sendFloatNumber(float number);
uint8_t MAX7219_sendFreq(float freq);
uint8_t MAX7219_sendDuty(float duty);
void MAX7219_sendString(char *str);
void MAX7219_sendDigit(uint8_t seg, uint8_t digit);
void MAX7219_clearAll();
void MAX7219_sendNumber(int number);
void MAX7219_sendOneChar(uint8_t seg, char c);
void MAX7219_sendOffsetString(uint8_t offset, char *str);
void MAX7219_clearOneSegment(uint8_t seg);



//#define CS_set()			HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET)		// ___/ *
//#define CS_reset()		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET)		// ---\ *
#define CS_set(CS_PORT, CS_PIN)			HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET)
#define CS_reset(CS_PORT, CS_PIN)		HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET)

#endif /* INC_MAX7219_H_ */
