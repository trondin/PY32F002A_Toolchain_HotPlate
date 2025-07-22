#include "py32f030x6.h"

// Pulse delay in percentage of half-period (0-100%)
#define PULSE_DELAY_PERCENT 50
// Pulse duration (100 µs in ticks at 48 MHz)
#define PULSE_DURATION_TICKS 4800 // 100 µs = 4800 ticks at 48 MHz

// Global variable to store measured half-period
volatile uint32_t half_period_ticks = 10000; // Initial value (10 ms for 50 Hz)

void TIM1_PWM_Init(void)
{
    // Enable clock for TIM1, GPIOA, and GPIOB
    RCC->APBENR2 |= RCC_APBENR2_TIM1EN; // Enable TIM1 clock
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN; // Enable GPIOA and GPIOB clock

    // Configure pins PB3 (TIM1_CH2), PA0 (TIM1_CH3), PA1 (TIM1_CH4), PA7 (TIM1_CH1)
    GPIOB->MODER &= ~(GPIO_MODER_MODE3); // Reset mode for PB3
    GPIOB->MODER |= GPIO_MODER_MODE3_1; // Alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE7); // Reset mode for PA0, PA1, PA7
    GPIOA->MODER |= (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1 | GPIO_MODER_MODE7_1); // Alternate function

    // AF for PB3
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL3; // Reset AF for PB3
    GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL3_Pos); // AF1 for TIM1_CH2

    // AF for PA0, PA1, PA7
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1 | GPIO_AFRL_AFSEL7); // Reset AF
    GPIOA->AFR[0] |= (13 << GPIO_AFRL_AFSEL0_Pos) | (13 << GPIO_AFRL_AFSEL1_Pos) | (2 << GPIO_AFRL_AFSEL7_Pos);  

    // Configure TIM1 for PWM at 16 kHz
    TIM1->ARR = 1000; 
    TIM1->PSC = 0; // Prescaler = 1 (no division)

    // Configure PWM mode for channels 1, 2, 3, 4
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M); // Reset modes for channels 1 and 2
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1PE); // PWM Mode 1 + preload for CH1
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE); // PWM Mode 1 + preload for CH2
    TIM1->CCMR2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC4M); // Reset modes for channels 3 and 4
    TIM1->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3PE); // PWM Mode 1 + preload for CH3
    TIM1->CCMR2 |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE); // PWM Mode 1 + preload for CH4

    // Enable channel outputs
    TIM1->CCER |= TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // Set duty cycle
    TIM1->CCR1 = 150; // 10% for CH1 (PA7 NE)
    TIM1->CCR2 = 375; // 25% for CH2 (PB3)
    TIM1->CCR3 = 750; // 50% for CH3 (PA0)
    TIM1->CCR4 = 900; // 90% for CH4 (PA1)

    // Enable auto-preload and timer
    TIM1->CR1 |= TIM_CR1_ARPE; // Enable auto-preload
    TIM1->BDTR |= TIM_BDTR_MOE; // Enable main outputs
    TIM1->CR1 |= TIM_CR1_CEN; // Enable TIM1
}