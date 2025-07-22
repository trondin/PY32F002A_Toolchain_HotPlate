#include "py32f030x6.h"
#include "systick.h"
#include "ntc_lut.h"

volatile uint16_t ntc_adc_value = 0; // Global variable to store ADC value

// Rup=5.1k
// Temperature lookup
const uint16_t lookup_ntc[][2]={
3994,	100,
3967,	150,
3935,	200,
3896,	250,
3851,	300,
3797,	350,
3735,	400,
3665,	450,
3585,	500,
3496,	550,
3397,	600,
3290,	650,
3174,	700,
3050,	750,
2920,	800,
2785,	850,
2646,	900,
2505,	950,
2363,	1000,
2222,	1050,
2083,	1100,
1947,	1150,
1815,	1200,
1688,	1250,
1568,	1300,
1453,	1350,
1345,	1400,
1244,	1450,
1149,	1500,
1061,	1550,
980,	1600,
904,	1650,
835,	1700,
770,	1750,
711,	1800,
657,	1850,
607,	1900,
562,	1950,
520,	2000,
482,	2050,
446,	2100,
414,	2150,
385,	2200,
};

void ADC_Init(void) {
    // Clock configuration for GPIOA, ADC, and TIM14
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;    // Enable GPIOA clock
    RCC->APBENR2 |= RCC_APBENR2_ADCEN | RCC_APBENR2_TIM14EN;    // Enable clock
    GPIOA->MODER |= (3 << 14);  // PA7 in analog input mode (MODER7 = 11)

    RCC->APBRSTR2 |= RCC_APBRSTR2_ADCRST;     // Reset ADC
    RCC->APBRSTR2 &= ~RCC_APBRSTR2_ADCRST;   
    // ADC configuration
    ADC1->CFGR2 = 0x0A << 28;  // HSI/4
   
// 3. ADC configuration
    ADC1->CFGR2 = (0x0A << 28);                  // ADC frequency: HSI/4 (48 MHz / 4 = 12 MHz)
    ADC1->CFGR1 = 0;                           // 12-bit right-aligned, single conversion
    ADC1->SMPR = (4 << 0);                    // Sampling time: 47.5 cycles
    ADC1->CHSELR = ADC_CHSELR_CHSEL7;         // Select channel 7 (PA7)

    ADC1->CR |= ADC_CR_ADCAL;                 // ADC calibration
    while (ADC1->CR & ADC_CR_ADCAL);           

    ADC1->CR |= ADC_CR_ADEN;                 
    delay_ms(2);     

    // Enable interrupt on conversion completion
    ADC1->IER |= ADC_IER_EOCIE;                
    NVIC_SetPriority(ADC_COMP_IRQn, 1);        
    NVIC_EnableIRQ(ADC_COMP_IRQn);             

    // Configure TIM14 for 10 Hz
    #define ADC_PERIOD 10       
    TIM14->PSC = 4799;               
    TIM14->ARR = (SystemCoreClock / (TIM14->PSC + 1) / ADC_PERIOD) - 1; // Period for 10 Hz (9999 at 10 kHz)
    TIM14->DIER |= TIM_DIER_UIE;      // Enable update interrupt
    TIM14->CR1 |= TIM_CR1_CEN;        // Enable timer
    NVIC_EnableIRQ(TIM14_IRQn);
    NVIC_SetPriority(TIM14_IRQn, 1);
}

// ADC interrupt handler
void ADC_COMP_IRQHandler(void)
{ 
  static uint16_t adc_samples[3] = {240, 240, 240}; // Initial values: 24°C
  static uint8_t sample_index = 0;
  if (ADC1->ISR & ADC_ISR_EOC) 
  {
    uint16_t adc_value = ADC1->DR;     
    // Median filter for three values    
    adc_samples[sample_index] = adc_value;
    sample_index = (sample_index + 1) % 3;
    uint16_t median_adc;    
    uint16_t a = adc_samples[0];
    uint16_t b = adc_samples[1];
    uint16_t c = adc_samples[2];
    median_adc = (a >= b) ? 
      ((b >= c) ? b : (a >= c) ? c : a) :
      ((a >= c) ? a : (b >= c) ? c : b);
    // Exponential filter 
    ntc_adc_value = ((ntc_adc_value * 7 + median_adc) >> 3);
    ADC1->ISR = ADC_ISR_EOC;    
    ADC1->CR &= ~ADC_CR_ADEN;    
    ADC1->CR |= ADC_CR_ADEN;  
  }
}

// TIM14 interrupt handler
void TIM14_IRQHandler(void)
{
  if (TIM14->SR & TIM_SR_UIF) 
  {
    ADC1->CR |= ADC_CR_ADSTART;  
    TIM14->SR &= ~TIM_SR_UIF; 
  }
}
 

// return temperature degree*10
int16_t analog2temp(uint16_t adc)
{
  int size = sizeof(lookup_ntc)/4;
  if (adc<=lookup_ntc[size-1][0]) return (lookup_ntc[size-1][1]+5)/10;
  if (adc>=lookup_ntc[0][0])      return (lookup_ntc[0][1]+5)/10;

  for (uint8_t i = 1; i < size; i++) 
  {
    if (lookup_ntc[i][0] < adc)
    {
      // interpolation temp = y0 + (x - x0) * (y1 - y0) / (x1 - x0)
      int16_t celsius = 0;
      celsius = lookup_ntc[i - 1][1] +
                (lookup_ntc[i - 1][0]-adc) *
                (lookup_ntc[i][1] -lookup_ntc[i - 1][1]) /
                (lookup_ntc[i-1][0] - lookup_ntc[i][0]);
      return celsius;
    }
  }
  return 0;
}


