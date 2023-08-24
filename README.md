# Генератор сигналов на stm32f401

Часто для тестирования каких-либо самодельных или готовых схем необходимо подавать сигнал, форма и параметры которого заранее известны с заданной точностью. Ещё чаще под рукой не оказывается нужных компонентов для воплощения задуманного - именно так и был создан этот проект. Было бы проще все сделать на каком-нибудь `stm32f407` с аппаратным ЦАПом и параметры такого устройства были бы лучше, но в наличии были только отладочные платы BlackPill с `stm32f401` на борту. Точности в 8 бит хватало, а между максимальной частотой выдаваемых сигналов и точностью ЦАПа была выбрана частота.

Разработка программной части происходила в `STM32CubeIDE`.
Разработка печатной платы, а также всей необходимой документации велась в САПР `Altium Designer`. 

___
## Содержание
- [Генератор сигналов на stm32f401](#генератор-сигналов-на-stm32f401)
  - [Содержание](#содержание)
  - [Характеристики](#характеристики)
  - [Программная часть:](#программная-часть)
    - [Концепция:](#концепция)
    - [Защита от дребезга:](#защита-от-дребезга)
    - [Работа с клавиатурой:](#работа-с-клавиатурой)
    - [Остановка генерации:](#остановка-генерации)
    - [Работа с индикатором и меню:](#работа-с-индикатором-и-меню)
  - [Аппаратная часть:](#аппаратная-часть)
    - [Клавиатура:](#клавиатура)
    - [Дисплей:](#дисплей)
    - [Электричская принципиальная схема:](#электричская-принципиальная-схема)
    - [Сборочный чертёж платы:](#сборочный-чертёж-платы)
    - [Чертеж, содержащий таблицу сверления:](#чертеж-содержащий-таблицу-сверления)
    - [Чертёж, содержащий трассировку печатной платы:](#чертёж-содержащий-трассировку-печатной-платы)
    - [Трассровка TOP:](#трассровка-top)
    - [Трассровка BOTTOM (Mirrored):](#трассровка-bottom-mirrored)
  - [Генератор в процессе отладки:](#генератор-в-процессе-отладки)
    - [Макет:](#макет)
    - [Отображение надписей:](#отображение-надписей)
___

## Характеристики
Данный генератор может выдавать ШИМ сигнал с заданной частотой и скважностью, пилообразный, как обратный, так и прямой, треугольный и синусоидальный.

- Разрядность ЦАПа - `8 бит`
- Погрешность частоты ЦАПа - `2%`
- Шаг задаваемой частоты - `100 Гц`

1. ШИМ сигнал:
   - Диапазон частоты: 0.1 - 999.9 кГц
   - Диапазон скважности: 0 - 100%
2. Пила и обратная пила:
   - Имеет 51 значений
   - Диапазон частот: 0.1 - 90 кГц
3. Треугольный сигнал:
   - Имеет 102 значений
   - Диапазон частот: 0.1 - 50 кГц
4. Синусоидальный сигнал:
   - Имеет 256 значений
   - Диапазон частот: 0.1 - 20 кГц
  
___
## Программная часть:
### Концепция:
Программная часть разработана с использованием операционной системы `FreeRTOS`. Вся программа сводится к 4 главным задачам: задачи работы с клавиатурой, для работы с дисплеем, а также задачи управления ШИМ сигналом и ЦАПом. 

```C
void readButtonTask(void const * argument);
void displayTask(void const * argument);
void dacTask(void const * argument);
void pwmTask(void const * argument);
```

Для реализации **ЦАПа** использован `DMA (Memory to GPIO)` срабатывающий от таймера 1. Частота срабатывания DMA высчитывается исходя из введенной пользователем частоты, фиксированного шага ЦАПа, количества значений присущего конкретному сигналу, а также учитывая частоту таймера (64 МГц). Все вычисления происходят один раз при запуске ЦАПа, далее CPU не участвует в его работе за счёт DMA.

**ШИМ** сигнал формируется также аппаратно без участия CPU с помощью 32 битного `TIMER_5`.
___

### Защита от дребезга:

`Защита от дребезга` реализована программно: при срабатывании прерывания по заднему фронту в callback-функции этого события запускается таймер на `50 мс`: 
```C
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	
{
   button_exti = GPIO_Pin;	/* Фиксируем ножку, на к-ой было прерывание   */
   HAL_TIM_Base_Start_IT(&htim9);	
   ...
}
```

По прошествии этого времени, попадая в callback-функцию таймера, программа проверит, присутствует ли до сих пор сигнал на той ножке, которую мы зафиксировали ранее, и, если есть, считается, что это не случайное срабатывание и можно передать её номер для дальнейшей обработки: 

```C
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
   ...
   if(htim->Instance == TIM9)
	{
      ...
      if( HAL_GPIO_ReadPin(GPIOA, button_exti) == GPIO_PIN_RESET )	
      {
         uint16_t button_num = button_exti;
         switch(button_exti)
         {
            case BUTTON_LEFT_Pin:
               button_num = BUTTON_LEFT_Pin;
               break;
            ...
            case BUTTON_RETURN_Pin:
		button_num = BUTTON_RETURN_Pin;
		dac_is_running = false;				
		pwm_is_running = false;
		break;
         }
         xQueueSendToBackFromISR(xButtonQueue, &button_num, &high_task_awoken);
         /*  Отрпавим номер ножки в очередь  */
      }
      ...
   }
   ...
}
```
____
### Работа с клавиатурой:

В задаче `readButtonTask` происходит обработка нажатий клавиатуры в switch-case конструкции. Номер нажатой кнопки задача получает из очереди. Помещаются данные в эту очередь из callback-функции таймера, если нажатие было "устойчивым".

```C
xQueueReceive(xButtonQueue, &button_num, 100 / portTICK_RATE_MS);
switch(button_num)
{
   case BUTTON_LEFT_Pin:
      ...
      break;
   case BUTTON_RIGHT_Pin:
      ...
      break;
      ...
      ...
      ...
   case BUTTON_RETURN_Pin:
      ...
      break;
}
```
___
### Остановка генерации:

Остановка генерации ЦАП или ШИМ сигнала происходит по одному принципу и код их идентичен - как только в callback-функции таймера мы попадаем в `case` кнопки `RETURN` (это произойдёт, если мы действительно нажали кнопку RETURN),  опускаются флажки состояния `dac_is_running` и `pwm_is_running`. Задачи по работе с ЦАПом и ШИМ имеют высший приортитет среди других задач, поэтому, как только проходит период в 100 мс, задача проверит состояние флажка и сразу остановит генерацию.

```C
void dacTask(void const * argument)
{
  /* USER CODE BEGIN dacTask */
  ...
  for(;;)
  {
      	...
	if( !dac_is_running && dac_is_started)	
	{
		DAC_stop();
		dac_is_started = false;
	}
	vTaskDelay(100 / portTICK_RATE_MS);
  }
}
```

Таким образом имеем гаранитрованное время (не более `150 мс` после нажатия), через которое генерация любого сигнала прекращается.
___
### Работа с индикатором и меню:

Для работы с 8-разрядным индикатором на микросхеме MAX7219 и реалзиации пользовательского интерфейса, был написан драйвер, в котором реализован "прицельный" вывод цифр и символов, вывод строк, целочисленных чисел, чисел с плавающей точкой, и т.д. 

Структура меню, отображаемой на индикаторе, выглядит следующим образом:

![Блок-схема меню генератора](https://i.postimg.cc/mDF5D5pH/image.jpg,  "Блок-схема меню генератора")

Управляется генератор с клавиатуры.
___
## Аппаратная часть:
### Клавиатура:

Для управления генератором было выбрано следующее расположение кнопок:

![Клавиатура управления](https://i.postimg.cc/JztKVHwt/image.png,  "Клавиатура управления")

- Кнопки &larr; и &rarr; используются для выбора типа сигнала ЦАПа и для выбора разряда в числе (десятые, единицы, десятки и т.д), который изменяется пользователем
- Кнопки &uarr; и &darr; используются чтобы увеличивать или уменьшать выбранный разряд в числе (&uarr; - увеличивает выбранный разряд, &darr; - уменьшает)
- Кнопка **OK** подтверждает введенные параметры - режим работы, тип сигнала, введенную частоту и скважность и переносит тем самым пользователя на следующий этап ввода настроек или же на стадию, когда генератор вырабатывает сигнал
- Кнопка **RET** возвращает программу в самое начало на стадию выбора режима (PWM или DAC), останавливая генерацию сигнала, если генератор уже запущен.

Все кнопки подтянуты внутренними резисторами микроконтроллера к питанию.
___
### Дисплей:
Для отображения информации используется модуль со встроенным драйвером для 8-разрядного индикатора (MAX7219). Драйвер общается с МК по SPI.
  
![8-разрядный индикатор](https://i.postimg.cc/zBpbd3Pk/8.jpg,  "8-разрядный индикатор")
___

### Электричская принципиальная схема:

![Электрическая принципиальная схема](https://i.postimg.cc/Wbhpt5qy/Electrical-Scheme.png,  "Электрическая принципиальная схема")

Назначение разъёмов и клемм:
- Разъём P1 нужен для прошивки микроконтроллера программатором ST-link
- На клеммы P2 приходит выпрямленное постоянное напряжение в диапазоне 7-18V.
- С клемм P3 снимается сгенерированный сигнал.
- Разъём P4 уже соединен со всеми нужными выводами микроконтроллера. На этот разъём сажается 8-разрядный индикатор
- Разъём P5 никуда не подключен и служит лишь для крепления 8-разрядного индикатора на плату

ЦАП построен на R-2R цепочке с использованием чип-резисторов точностью 1%.

Чтобы не нагромождать плату генератора трансформатором и выпрямителем, было решено проектировать плату без силовой части. Также на выходе рекомендуется ставить буффер с низким выходным сопротивлением, например схему `повторителя на ОУ`. 
___

### Сборочный чертёж платы:

![8-разрядный индикатор](https://i.postimg.cc/Vk4WvXXf/Draftsman-page-0001.jpg, "Сборочный чертёж платы")

Зеленым цветом отмечен слой контур индикатора, который размещается непосрдественно на плату в виде модуля, а также отмечена распиновка входной (P2) и выходной (P3) клеммы. На данном чертеже не отображена 3D модель самого модуля индикатора, но его высота **не превышает** высоты электролитического конденсатора на C4 (12 +- 0.5 мм).
___

### Чертеж, содержащий таблицу сверления:
![Чертеж, содержащий таблицу сверления](https://i.postimg.cc/5t0qCjmw/Draftsman-page-0002.jpg, "Чертеж, содержащий таблицу сверления")
___

### Чертёж, содержащий трассировку печатной платы:
![Чертёж, содержащий трассировку печатной платы](https://i.postimg.cc/1zj0cPWb/Draftsman-page-0003.jpg, "Чертёж, содержащий трассировку печатной платы")
___

### Трассровка TOP:
![Top](https://i.postimg.cc/6qYpn6cv/PCB-signal-Gen-page-0001.jpg, "Top")
___

### Трассровка BOTTOM (Mirrored):
![Bottom](https://i.postimg.cc/pr2dm5fv/Bottom-track-page-0001-1.jpg, "Bottom")

____
## Генератор в процессе отладки:
### Макет:
![Генератор сигналов, макет](https://i.postimg.cc/g2V6K3vm/image.jpg, "Генератор сигналов, макет")
___

### Отображение надписей:
![Режим ЦАПа](https://i.postimg.cc/y8c0rMzq/dac-mode.jpg, "Режим ЦАПа")
