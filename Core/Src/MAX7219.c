#include "MAX7219.h"

static max7219_init_t *MAX7219_Handler;	

static HAL_StatusTypeDef MAX7219_write(uint8_t reg, uint8_t data)
{
	HAL_StatusTypeDef result;
	uint8_t value[2];
	value[0] = reg;
	value[1] = data;

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);	// Строб --\__		
	result = HAL_SPI_Transmit(&hspi1, value, 2, 1000);	
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);		// Строб __/
	return result;
}
//---------------------------------------------------------
static HAL_StatusTypeDef MAX7219_setDecode(uint8_t mode)
{
	if(MAX7219_Handler->decode_mode != mode)	
	{
		MAX7219_Handler->decode_mode = mode;		/* Присовим новый режим, чтобы его потом отслеживать	*/
		return ( MAX7219_write(MAX7219_DECODE_MODE_REG, mode) );	
	}
	return HAL_OK;
}
//---------------------------------------------------------
void MAX7219_sendDigit(uint8_t seg, uint8_t digit)
{
	MAX7219_write(seg, digit);
}
//---------------------------------------------------------
static char MAX7219_convertSymbol(char symbol) {
	switch(symbol) {
		case 0: return _0;
		break;
		case 1: return _1;
		break;
		case 2: return _2;
		break;
		case 3: return _3;
		break;
		case 4: return _4;
		break;
		case 5: return _5;
		break;
		case 6: return _6;
		break;
		case 7: return _7;
		break;
		case 8: return _8;
		break;
		case 9: return _9;
		break;
		case '0': return _0;
		break;
		case '1': return _1;
		break;
		case '2': return _2;
		break;
		case '3': return _3;
		break;
		case '4': return _4;
		break;
		case '5': return _5;
		break;
		case '6': return _6;
		break;
		case '7': return _7;
		break;
		case '8': return _8;
		break;
		case '9': return _9;
		break;
		case 'A': return _A;
		break;
		case 'B': return _B;
		break;
		case 'C': return _C;
		break;
		case 'D': return _D;
		break;
		case 'E': return _E;
		break;
		case 'F': return _F;
		break;
		case 'G': return _G;
		break;
		case 'H': return _H;
		break;
		case 'I': return _I;
		break;
		case 'J': return _J;
		break;
		case 'K': return _K;
		break;
		case 'L': return _L;
		break;
		case 'M': return _M;
		break;
		case 'N': return _N;
		break;
		case 'O': return _O;
		break;
		case 'P': return _P;
		break;
		case 'Q': return _Q;
		break;
		case 'R': return _R;
		break;
		case 'S': return _S;
		break;
		case 'T': return _T;
		break;
		case 'U': return _U;
		break;
		case 'V': return _V;
		break;
		case 'W': return _W;
		break;
		case 'X': return _X;
		break;
		case 'Y': return _Y;
		break;
		case 'Z': return _Z;
		break;
		case '-': return _MINUS;
		break;
		default: return 0;
	}
}
//---------------------------------------------------------
void MAX7219_sendOneChar(uint8_t seg, char c)
{
	MAX7219_write(seg, MAX7219_convertSymbol( c ));			
}
//---------------------------------------------------------
void MAX7219_sendOffsetString(uint8_t offset, char *str)
{
	int seg_iter = MAX7219_SEGMENTS_AMOUNT;		

	if (strlen(str) > 8) return;
	MAX7219_setDecode(MAX7219_NO_DECODE);
	for(uint8_t i = 0; str[i] != '\0'; i++)
	{
		if((seg_iter - offset) == 0) break;		/* Чтобы случайно не записать в другой регистр	*/
		MAX7219_sendOneChar( (seg_iter - offset), str[i] );
		seg_iter--;
	}
}
//---------------------------------------------------------
void MAX7219_sendString(char *str)
{
	int seg_iter = 8;							
	if (strlen(str) > 8) return;				/* Проверка на валидность строки (не более 8 символов)	*/
	MAX7219_setDecode(MAX7219_NO_DECODE);		
	for(uint8_t i = 0; str[i] != '\0'; i++)
	{
		MAX7219_sendOneChar( seg_iter, str[i]);
		seg_iter--;
	}
}
//---------------------------------------------------------
void MAX7219_sendErrMsg()
{
	MAX7219_sendString("Error");
}
//---------------------------------------------------------
uint8_t MAX7219_sendFloatNumber(float number)
{
	int seg_iter = MAX7219_SEGMENTS_AMOUNT;
	char str[9];
	sprintf(str, "%.1f", number);	// Завернули в строку

	for(uint8_t i = 0; str[i] != '\0'; i++)
	{
		if( str[i+1] == '.') {
			MAX7219_write( seg_iter, MAX7219_convertSymbol( str[i] ) | POINT );	/* Добавим 0х80 чтобы вывести в тот же сегмент точку	*/
		}
		else if(str[i] == '.') {
			continue;
		}
		else {
			MAX7219_sendOneChar( seg_iter, str[i] );
		}
		seg_iter--;
	}
	return (strlen(str));
	/* Возврат величины нужен для того, чтобы знать смещение и приписать размерность числу (205.5 Hz к примеру).
	  	  Используется для функции MAX7219_sendOffsetString() */
}
//---------------------------------------------------------
void MAX7219_sendNumber(int number)
{
	if(number == 0) {
		MAX7219_sendOneChar(MAX7219_SEGMENT_7 , 0);	
		return;
	}

	int k = 0;					
	uint8_t buffer[8] = {};		

	if(number > 0)		
	{
		for (int i = 7; i >= 0; i--)
		{
			if(number > 0)
			{
				buffer[i] = number % 10;		/* Записываем число с конца, а затем получем массив цифр ида: number = 107 =>	*/
				number /= 10;					/* buffer[4] = [0, 1, 0, 7];	*/
			}									 
			else break;
		}
		for (uint8_t i = 0; i < 8; i++)		
		{
			if(buffer[i] == 0 && k == 0) continue;
			/*
				Как только появится первое число в массиве, подлежащее записи, то мы попадём в ветку else,
				и k уже больше не будет равен нулю, поэтому это можно считатать признаком начала числа
				и использовать в условии пропуска, т.к просто условия buffer[i] == 0 недостотчно, ибо в числе
				могут присутствовать нули и они просто не выведутся на экран
			*/
			else
			{
				MAX7219_sendOneChar( (MAX7219_SEGMENT_7 - k), buffer[i] );
				k++;
				/*
					k увеличивается только тогда, когда в массиве НЕ ноль, это позволяет писать число
					с первого сегмента на дисплей, в то время как интерация по массиву чисел происходит обычным
					образом включая нули.
				*/
			}
		}
	}
	else {		/* Если число отрицательное	*/
		MAX7219_sendOneChar(MAX7219_SEGMENT_7, '-');	
		number = number - (2*number);			/* Делаем из отрицательного положиетльное	*/
		for (int i = 7; i >= 0; i--)
		{
			if(number > 0)
			{
				buffer[i] = number % 10;
				number /= 10;
			}
		}
		for (uint8_t i = 0; i < 8; i++)
		{
			if(buffer[i] == 0 && k == 0) continue;
			else
			{
				MAX7219_sendOneChar( (MAX7219_SEGMENT_6 - k), buffer[i] );
				k++;
			}
		}
	}
}
//---------------------------------------------------------
uint8_t MAX7219_sendDuty(float duty)
{
	uint8_t offset;
	offset = MAX7219_sendFloatNumber(duty);
	return offset;
}
//---------------------------------------------------------
uint8_t MAX7219_sendFreq(float freq)
{
	uint8_t offset;
	offset = MAX7219_sendFloatNumber(freq);
	MAX7219_sendOffsetString(offset, "KHZ");
	return offset;
}
//---------------------------------------------------------
void MAX7219_clearOneSegment(uint8_t seg)
{
	MAX7219_sendOffsetString(seg, " ");
}
void MAX7219_blinkOneSegment(uint8_t seg)
{

}
void MAX7219_clearAll()
{
	for (uint8_t i = MAX7219_SEGMENT_0; i <= MAX7219_SEGMENT_7; i++)
	{
 		MAX7219_write(i, 0);
	}
}
//---------------------------------------------------------
HAL_StatusTypeDef MAX7219_init(max7219_init_t *cfg)
{
	HAL_StatusTypeDef result;
	MAX7219_Handler = cfg;
	MAX7219_write(MAX7219_DECODE_MODE_REG, MAX7219_Handler->decode_mode);
	MAX7219_write(MAX7219_INTENSITY_REG, MAX7219_Handler->indicator_intensity);
	MAX7219_write(MAX7219_SCAN_LIMIT_REG, MAX7219_Handler->digits_quantity);
	result = MAX7219_write(MAX7219_MODE_REG, MAX7219_NORMAL_MODE);
	MAX7219_clearAll();		
	return result;
}



