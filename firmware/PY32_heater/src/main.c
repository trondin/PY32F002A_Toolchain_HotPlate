#include "py32f030x6.h"
#include "uart.h"
#include "systick.h"
#include <stdio.h>
#include <stdlib.h>
#include "lcd.h"
#include "scr_power.h"
#include "ntc_lut.h"
#include "eeprom_emul.h"
#include "pid.h"

// Константы времени (в миллисекундах)
#define PID_UPDATE_INTERVAL       100     // Интервал обновления PID
#define BLINK_INTERVAL            200     // Интервал мигания светодиода
#define EEPROM_UPDATE_INTERVAL    60000   // Интервал обновления EEPROM (60 сек)
#define LCD_UPDATE_INTERVAL       200     // Интервал обновления LCD
#define KEYBOARD_POLL_INTERVAL    100     // Интервал опроса клавиатуры

// Настройки автоинкремента/автодекремента
#define AUTO_CHANGE_DELAY        2000    // Задержка до быстрого изменения (3 сек)
#define AUTO_CHANGE_INTERVAL     300     // Интервал автоизменения (0.5 сек)

// Пределы температуры
#define MIN_TEMPERATURE          200     // Минимальная температура (20.0°C)
#define MAX_TEMPERATURE          2200    // Максимальная температура (220.0°C)
#define DEFAULT_TEMPERATURE      900     // Температура по умолчанию (90.0°C)

extern uint16_t ntc_adc_value;

// Структура для хранения состояния системы
typedef struct {
  // Временные метки
  uint32_t last_blink_time;
  uint32_t last_pid_time;
  uint32_t last_eeprom_update;
  uint32_t last_lcd_update;
  uint32_t last_kbr_update;
    
  // Состояние системы
  uint16_t tempPoint;
  uint8_t PowerScr;
  uint16_t ntc_temper;
  uint16_t lcd_temper;
  uint8_t ActiveMode;
    
  // Состояния для обнаружения изменений
  uint8_t PreviousMode;
  uint8_t prev_lcd_temper;
  int8_t PrevLcdTempSet;
  uint8_t PreviousPower;
  uint8_t keys_en;
    
  // Состояние автоизменения температуры
  uint32_t button_press_time;
  uint32_t last_auto_change_time;
  uint8_t is_auto_changing;
  uint8_t auto_change_direction; // 0 - нет, 1 - увеличение, 2 - уменьшение
} SystemState;


// Инициализация кнопок
void keys_init(void)
{
  // Enable clocks for GPIOA, GPIOB
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

  // PA13 (BTN0 - down/off)
  GPIOA->MODER &= ~GPIO_MODER_MODE13;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD13_0; // Pull-up
  GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL13;

  // PB3 (BTN1 - right/plus)
  GPIOB->MODER &= ~GPIO_MODER_MODE3;
  GPIOB->PUPDR |= GPIO_PUPDR_PUPD3_0; // Pull-up

  // PA1 (BTN2 - up/on)
  GPIOA->MODER &= ~GPIO_MODER_MODE1;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD1_0; // Pull-up

  // PA4 (BTN3 - left/minus)
  GPIOA->MODER &= ~GPIO_MODER_MODE4;
  GPIOA->PUPDR |= GPIO_PUPDR_PUPD4_0; // Pull-up
}

// Сканирование состояния кнопок
uint8_t keys_scan(void) 
{
  return ((~(GPIOA->IDR) & GPIO_IDR_ID13) >> 13) |  // BTN0 (PA13) -> bit 0
         ((~(GPIOB->IDR) & GPIO_IDR_ID3)  >> 2)  |  // BTN1 (PB3)  -> bit 1
         ((~(GPIOA->IDR) & GPIO_IDR_ID1)  << 1)  |  // BTN2 (PA1)  -> bit 2
         ((~(GPIOA->IDR) & GPIO_IDR_ID4)  >> 1);    // BTN3 (PA4)  -> bit 3
}

// Обновление LCD в активном режиме
void update_lcd_active_mode(SystemState *state) 
{
  uint8_t snum[5];
  int8_t LcdTempSet = (state->tempPoint + 5) / 10;
    
  if(state->PrevLcdTempSet != LcdTempSet) 
  {
    LCD_SetCursor(4, 0);
    LCD_WriteString("    ");        
    itoa(LcdTempSet, snum, 10);
    LCD_SetCursor(4, 0);                
    LCD_WriteString(snum);      
    state->PrevLcdTempSet = LcdTempSet;
  }
    
  if(state->PreviousPower != state->PowerScr) 
  {
    LCD_SetCursor(4, 3);  
    LCD_WriteString("    ");  
    itoa(state->PowerScr, snum, 10);  
    LCD_SetCursor(4, 3);  
    LCD_WriteString(snum);           
    state->PreviousPower = state->PowerScr;        
  }
}

void handle_buttons(SystemState *state) 
{
  uint8_t keys = keys_scan();
  uint32_t current_time = get_system_time();    
  
  // Обработка кнопок ON/OFF (мгновенное действие)
  if (keys & 0x01) // Кнопка OFF
  { 
    state->ActiveMode = 0;
    return;
  }

  if (keys & 0x04)  // Кнопка ON
  { 
    state->ActiveMode = 1;
    state->prev_lcd_temper = 0;
    state->PrevLcdTempSet = 0;
    state->PreviousPower = 100;
    return;
  }
    
  // Обработка кнопок +/-
  if (keys & 0x02)  // Кнопка +
  { 
    if (state->button_press_time == 0)  // Первое нажатие
    {
      state->tempPoint += 10;
      if (state->tempPoint > MAX_TEMPERATURE) state->tempPoint = MAX_TEMPERATURE;
      state->button_press_time = current_time;
      state->last_auto_change_time = current_time;
      EEPROM_Write(state->tempPoint);
    }
    // Автоизменение после задержки
    else 
    {
      if (current_time - state->button_press_time > AUTO_CHANGE_DELAY) 
      {
        if (current_time - state->last_auto_change_time > AUTO_CHANGE_INTERVAL) 
        {
          state->tempPoint += 10;
          if (state->tempPoint > MAX_TEMPERATURE) state->tempPoint = MAX_TEMPERATURE;
          state->last_auto_change_time = current_time;
          EEPROM_Write(state->tempPoint);
        }
      }
    }
  }
  else
  { 
    if (keys & 0x08)// Кнопка -
    {         
      if (state->button_press_time == 0) // Первое нажатие
      {
        state->tempPoint -= 10;
        if (state->tempPoint < MIN_TEMPERATURE) state->tempPoint = MIN_TEMPERATURE;
        state->button_press_time = current_time;
        state->last_auto_change_time = current_time;
        EEPROM_Write(state->tempPoint);
      }
      // Автоизменение после задержки
      else
      {
        if(current_time - state->button_press_time > AUTO_CHANGE_DELAY) 
        {
          if (current_time - state->last_auto_change_time > AUTO_CHANGE_INTERVAL) 
          {
            state->tempPoint -= 10;
            if (state->tempPoint < MIN_TEMPERATURE) state->tempPoint = MIN_TEMPERATURE;
            state->last_auto_change_time = current_time;
            EEPROM_Write(state->tempPoint);
          }
        }
      }
    }  
    else state->button_press_time = 0; // Кнопки отпущены - сброс таймера
    // Обновление времени последнего изменения EEPROM
    if (keys & (0x02 | 0x08)) state->last_eeprom_update = current_time;    
  }
}


int main(void) 
{
  // Инициализация системы
  SystemCoreClockUpdate();
  SystemClock_Config();
  SysTick_Init();
    
  // Настройка тестового светодиода на PB0
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  GPIOB->MODER &= ~GPIO_MODER_MODE0;
  GPIOB->MODER |= GPIO_MODER_MODE0_0;
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT0;
    
  // Инициализация периферии
  uart_init();
  ScrPowerConfig();
  ADC_Init();
  keys_init();
  LCD_Init();
    
  // Инициализация PID и EEPROM
  AdaptivePIDState pid;
  pid_init(&pid);
  EEPROM_Init(DEFAULT_TEMPERATURE);
    
  // Инициализация состояния системы
  SystemState state = {
        .last_blink_time = get_system_time(),
        .last_pid_time = get_system_time(),
        .last_eeprom_update = get_system_time(),
        .last_lcd_update = get_system_time(),
        .last_kbr_update = get_system_time(),
        .tempPoint = EEPROM_Read(),
        .PowerScr = 0,
        .ntc_temper = 0,
        .lcd_temper = 0,
        .ActiveMode = 0,
        .PreviousMode = 1,
        .prev_lcd_temper = 0,
        .PrevLcdTempSet = 0,
        .PreviousPower = 100,
        .keys_en = 0,
        .button_press_time = 0
  
    };

  while (1) 
  {
    uint32_t current_time = get_system_time();
        
    // Обновление PID каждые PID_UPDATE_INTERVAL мс
    if (current_time - state.last_pid_time >= PID_UPDATE_INTERVAL) 
    {
      state.ntc_temper = analog2temp(ntc_adc_value);
      state.PowerScr = pid_compute(&pid, state.tempPoint, state.ntc_temper);
      SetPower(state.ActiveMode ? state.PowerScr : 0);
      state.last_pid_time = current_time;
    }
        
    // Мигание светодиодом и вывод в UART каждые BLINK_INTERVAL мс
    if (current_time - state.last_blink_time >= BLINK_INTERVAL) 
    {
      GPIOB->ODR ^= 0x01;
      printf("%d,%d\n", (state.ntc_temper + 5) / 10, state.PowerScr);
      delay_ms(2);
      //USART1->DR = '*';
      state.last_blink_time = current_time;
    }
        
    // Обновление EEPROM каждые EEPROM_UPDATE_INTERVAL мс
    if (current_time - state.last_eeprom_update >= EEPROM_UPDATE_INTERVAL) 
    {
      EEPROM_Update();
      state.last_eeprom_update = current_time;
    }
        
    // Обновление LCD каждые LCD_UPDATE_INTERVAL мс
    if (current_time - state.last_lcd_update >= LCD_UPDATE_INTERVAL) 
    {
      if (state.PreviousMode != state.ActiveMode) 
      {
        if (!state.ActiveMode) 
        {
          LCD_SetCursor(0, 0);
          LCD_WriteString("T\xCCs=0    I");       
          LCD_SetCursor(0, 3);  
          LCD_WriteString("PWR=0    O");   
        }
        state.PreviousMode = state.ActiveMode;      
      }
            
      state.lcd_temper = (state.ntc_temper + 5) / 10;
      if (state.lcd_temper != state.prev_lcd_temper) 
      {
        Draw3Num(0, 1, state.lcd_temper);          
        state.prev_lcd_temper = state.lcd_temper;
      }
            
      if (state.ActiveMode) update_lcd_active_mode(&state);
            
      state.last_lcd_update = current_time;
    }
        
    // Обработка кнопок каждые KEYBOARD_POLL_INTERVAL мс
    if (current_time - state.last_kbr_update >= KEYBOARD_POLL_INTERVAL) 
    {
      handle_buttons(&state);
      state.last_kbr_update = current_time;
    }
  }
}