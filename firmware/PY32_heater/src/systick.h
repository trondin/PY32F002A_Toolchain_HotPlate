#ifndef SYSTICK_H
#define SYSTICK_H

#include <stdint.h>

void SystemClock_Config(void); 
void SysTick_Init(void);
void delay_ms(uint32_t ms);
uint32_t get_system_time(void);

#endif