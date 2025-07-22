#include "systick.h"
#include "py32f030x6.h"


#define HSI_FREQUENCY  24000000UL  // HSI frequency after configuration
#define PLL_MULTIPLIED_FREQ (HSI_FREQUENCY * 2)  // 48 MHz after PLL
void SystemClock_Config(void) 
{
    // Configure Flash (mandatory before increasing frequency!)
    FLASH->ACR |= FLASH_ACR_LATENCY;  // Enable wait state
    //while (!(FLASH->ACR & FLASH_ACR_LATENCY));  // Wait for application

    // Configure HSI to 24 MHz
    RCC->ICSCR = (RCC->ICSCR & ~RCC_ICSCR_HSI_FS_Msk) | (0x4 << RCC_ICSCR_HSI_FS_Pos);
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    // Enable PLL (multiplies HSI by 2)
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    // Configure dividers
    RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE_Msk);  // 1:1 for all buses
    // Switch to PLL
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0x02 << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0x02 << RCC_CFGR_SWS_Pos));
    SystemCoreClock = PLL_MULTIPLIED_FREQ;
}

// System tick counter
static volatile uint32_t system_ticks = 0;

// Initialize SysTick timer for 1ms interrupts
void SysTick_Init(void)
{
    // SystemCoreClock is 48 MHz (from main.c PLL configuration)
    // For 1ms tick: 48,000,000 / 1000 = 48,000 cycles
    SysTick->LOAD = (SystemCoreClock / 1000) - 1; // Set reload value for 1ms
    SysTick->VAL = 0; // Clear current value
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | // Use processor clock
                    SysTick_CTRL_TICKINT_Msk |   // Enable interrupt
                    SysTick_CTRL_ENABLE_Msk;     // Enable SysTick
    NVIC_SetPriority(SysTick_IRQn, 0); // Set highest priority
}

// SysTick interrupt handler
void SysTick_Handler(void)
{
    system_ticks++; // Increment tick counter
}

// Delay function using system ticks
void delay_ms(uint32_t ms)
{
    uint32_t start_ticks = system_ticks;
    while ((system_ticks - start_ticks) < ms)
    {
        __WFI(); // Wait for interrupt
    }
}

// Get current system time in milliseconds
uint32_t get_system_time(void)
{
    return system_ticks;
}
