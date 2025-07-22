#include "py32f030x6.h"
#include "scr_power.h"

#define NOMINAL_PERIOD  10000 // 10 ms (50 Hz, half-period)
#define MIN_PERIOD       8500 // 8.5 ms
#define MAX_PERIOD      11500 // 11.5 ms
//#define PULSE_DURATION_US 200 // 200 µs pulse duration
#define PULSE_DURATION_US 300 // 200 µs pulse duration
#define ZERO_DELAY_US 700 // 0.7 ms delay
#define DEBOUNCE_TIMEOUT_US 2300 // 2.3 ms timeout for debouncing

/*
Power to angle dependency table (2% step):
Power%	Angle (°)	F(θ)
----------------------------------------
0	0.00		0.000
2	26.45		0.020
4	33.62		0.040
6	38.77		0.060
8	42.98		0.080
10	46.60		0.100
12	49.84		0.120
14	52.79		0.140
16	55.53		0.160
18	58.10		0.180
20	60.54		0.200
22	62.86		0.220
24	65.09		0.240
26	67.24		0.260
28	69.33		0.280
30	71.36		0.300
32	73.34		0.320
34	75.28		0.340
36	77.19		0.360
38	79.07		0.380
40	80.92		0.400
42	82.76		0.420
44	84.58		0.440
46	86.40		0.460
48	88.20		0.480
50	90.00		0.500
52	91.80		0.520
54	93.60		0.540
56	95.42		0.560
58	97.24		0.580
60	99.08		0.600
62	100.93		0.620
64	102.81		0.640
66	104.72		0.660
68	106.66		0.680
70	108.64		0.700
72	110.67		0.720
74	112.76		0.740
76	114.91		0.760
78	117.14		0.780
80	119.46		0.800
82	121.90		0.820
84	124.47		0.840
86	127.21		0.860
88	130.16		0.880
90	133.40		0.900
92	137.02		0.920
94	141.23		0.940
96	146.38		0.960
98	153.55		0.980
100	180.00		1.000
*/

// Percentage of 180° for equal power increments (step ~2%)
static const uint8_t phase_delay_percent[] =
{
    100,  85,  81,  78,  76,  // 180.0°, 153.55°, 146.38°, 141.23°, 137.02°
     74,  72,  71,  69,  68,  // 133.4°, 130.16°, 127.21°, 124.47°, 121.9°
     66,  65,  64,  63,  61,  // 119.46°, 117.14°, 114.91°, 112.76°, 110.67°
     60,  59,  58,  57,  56,  // 108.64°, 106.66°, 104.72°, 102.81°, 100.93°
     55,  54,  53,  52,  51,  // 99.08°, 97.24°, 95.42°, 93.6°, 91.8°
     50,  49,  48,  47,  46,  // 90.0°, 88.2°, 86.4°, 84.58°, 82.76°
     45,  44,  43,  42,  41,  // 80.92°, 79.07°, 77.19°, 75.28°, 73.34°
     40,  39,  37,  36,  35,  // 71.36°, 69.33°, 67.24°, 65.09°, 62.86°
     34,  32,  31,  29,  28,  // 60.54°, 58.1°, 55.53°, 52.79°, 49.84°
     26,  24,  22,  19,  15,  // 46.6°, 42.98°, 38.77°, 33.62°, 26.45°
      0                   // 0.0°
};

volatile uint16_t last_tick = 0;
volatile uint16_t tick_diff = 0;
volatile uint16_t filtered_period = 10000; // Filtered period in µs (initially 10 ms)
volatile uint16_t old_period = 10000; 
volatile uint8_t power_level = 0; 
volatile uint8_t tim17_state = 0;

void ScrPowerConfig(void) 
{
  // *******************************************************************
  // TIM1 Config
  // *******************************************************************  

  // Enable clock for TIM1 and GPIOA
  RCC->APBENR2 |= RCC_APBENR2_TIM1EN;
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  // PA0 (TIM1_CH3) - output for pulse
  GPIOA->MODER &= ~GPIO_MODER_MODE0;
  GPIOA->MODER |= GPIO_MODER_MODE0_1; // Alternate function
  GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL0;
  GPIOA->AFR[0] |= (13 << GPIO_AFRL_AFSEL0_Pos); // AF13 for TIM1_CH3

  // Configure TIM1
  TIM1->PSC = 47; // Prescaler = 48 (48 MHz / 48 = 1 MHz, 1 tick = 1 µs)
  TIM1->ARR = 0xFFFF; // Initial maximum period for flexibility

  // Configure TIM1_CH3 channel for pulse generation on PA0
  TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
  TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM Mode 1 (OC3M = 110)
  TIM1->CCR3 = 0; // Initial delay
  TIM1->CCER |= TIM_CCER_CC3E; // Enable channel 3 output (PA0)

  // Enable One-Pulse Mode for generating a single pulse per trigger
  TIM1->CR1 |= TIM_CR1_OPM;
  // Enable main output
  TIM1->BDTR |= TIM_BDTR_MOE;

  // *******************************************************************
  // TIM16 Config
  // *******************************************************************    
  RCC->APBENR2 |= RCC_APBENR2_TIM16EN; // Enable clock for TIM16
  TIM16->PSC = 47; // Prescaler = 48 (48 MHz / 48 = 1 MHz, 1 tick = 1 µs)
  TIM16->ARR = 0xFFFF; // Maximum period for 16-bit timer
  TIM16->CR1 |= TIM_CR1_CEN; // Enable timer

  // *******************************************************************
  // TIM17 Config for Debouncing
  // *******************************************************************    
  RCC->APBENR2 |= RCC_APBENR2_TIM17EN; // Enable clock for TIM17
  TIM17->PSC = 47; // Prescaler = 48 (48 MHz / 48 = 1 MHz, 1 tick = 1 µs)
  TIM17->ARR = DEBOUNCE_TIMEOUT_US; // 3 ms timeout
  //TIM17->DIER |= TIM_DIER_UIE; // Enable update interrupt
  NVIC_SetPriority(TIM17_IRQn, 0); // Set priority
  NVIC_EnableIRQ(TIM17_IRQn); // Enable TIM17 interrupt

  // *******************************************************************
  // EXTI Config
  // *******************************************************************    
  RCC->APBENR2 |= RCC_APBENR2_SYSCFGEN;  
  RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
  // PB2 - input for EXTI2
  GPIOB->MODER &= ~GPIO_MODER_MODE2; // Input mode
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD2; // No pull-up/pull-down

  EXTI->EXTICR[0] &= ~EXTI_EXTICR1_EXTI2; // Clear pin selection
  EXTI->EXTICR[0] |= EXTI_EXTICR1_EXTI2_0; // PB2 for EXTI2

  EXTI->IMR |= EXTI_IMR_IM2; // Enable interrupt
  EXTI->RTSR &= ~EXTI_RTSR_RT2; // Rising edge
  EXTI->FTSR |= EXTI_FTSR_FT2; // Falling edge
  NVIC_SetPriority(EXTI2_3_IRQn, 0); // Set priority
  NVIC_EnableIRQ(EXTI2_3_IRQn); // Enable EXTI2 (lines 2-3)  
}

// TIM17 interrupt handler for debouncing
void TIM17_IRQHandler(void)
{
  if (TIM17->SR & TIM_SR_UIF)
  {
    TIM17->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    TIM17->CR1 &= ~TIM_CR1_CEN; // Stop timer
    if(tim17_state ==0)
    {
      tim17_state++;
      TIM17->CNT = 0; 
      TIM17->ARR = DEBOUNCE_TIMEOUT_US; // 2.3 ms timeout
      TIM17->CR1  |= TIM_CR1_CEN; 
      TIM17->DIER |= TIM_DIER_UIE;       
      // Start TIM1 timer
      TIM1->CR1 &= ~TIM_CR1_CEN; 
      TIM1->CNT = 0; 
      TIM1->CR1 |= TIM_CR1_CEN; 
    }
    else
    {
      EXTI->IMR |= EXTI_IMR_IM2; // Enable EXTI2 interrupt
      TIM17->DIER &= ~TIM_DIER_UIE; // Disable interrupt   
    }
  }
}

// EXTI2 (PB2) interrupt handler
void EXTI2_3_IRQHandler(void) 
{
  static uint16_t period_samples[3] = {10000, 10000, 10000}; // Initial values: 10 ms (50 Hz, half-period)
  static uint8_t sample_index = 0;
  if (EXTI->PR & EXTI_PR_PR2) 
  {
    uint16_t current_tick = TIM16->CNT;
    // Start TIM1 timer
    //TIM1->CR1 &= ~TIM_CR1_CEN; 
    //TIM1->CNT = 0; 
    //TIM1->CR1 |= TIM_CR1_CEN; 

    tick_diff = current_tick - last_tick;
    last_tick = current_tick;

    // Start TIM17 timer for debouncing
    EXTI->IMR &= ~EXTI_IMR_IM2; // Disable EXTI2 interrupt  
    tim17_state = 0;
    TIM17->CR1 &= ~TIM_CR1_CEN;     
    TIM17->CNT = 0; 
    TIM17->ARR = ZERO_DELAY_US; // 0.7 ms delay
    TIM17->CR1 |= TIM_CR1_CEN; // Enable TIM17 timer    
    TIM17->SR &= ~TIM_SR_UIF; // Clear interrupt flag
    TIM17->DIER |= TIM_DIER_UIE; // Enable interrupt

    if (tick_diff >= MIN_PERIOD && tick_diff <= MAX_PERIOD)
    {
      period_samples[sample_index] = tick_diff;
      sample_index = (sample_index + 1) % 3;
      // Median filter for three values
      uint16_t median_period;    
      uint16_t a = period_samples[0];
      uint16_t b = period_samples[1];
      uint16_t c = period_samples[2];
      median_period = (a >= b) ? 
        ((b >= c) ? b : (a >= c) ? c : a) :
        ((a >= c) ? a : (b >= c) ? c : b);
      // Exponential filter 
      filtered_period = ((filtered_period * 3 + median_period) >> 2);
      // Calculate delay for TIM1 
      if (filtered_period != old_period)
      {
        uint32_t delay = (uint32_t)filtered_period * phase_delay_percent[power_level] / 100;
        old_period = filtered_period;
        TIM1->CCR3 = delay; 
        TIM1->ARR = delay + PULSE_DURATION_US; 
      }
    }
    EXTI->PR |= EXTI_PR_PR2; 
  }
}

void SetPower(uint8_t power)
{
  if (power > 50) return;
  power_level = power;
  old_period = 0; // Force timer settings update
}