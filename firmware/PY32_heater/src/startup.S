/*
 * File: startup.S
 * Description: Startup code for PY32F030F26P6 (Flash 32K, SRAM 4K)
 * Architecture: ARM Cortex-M0+
 * Compiler: GCC
 */

.syntax unified
.cpu cortex-m0
.thumb

/* Stack Configuration */
.equ Stack_Size, 0x00000200  /* 512 bytes for stack, matches py32f030.ld */
.section .stack, "w", %nobits
.align 3
.global __initial_sp
Stack_Mem:
.space Stack_Size
__initial_sp:

/* Heap Configuration */
.equ Heap_Size, 0x00000080  /* 128 bytes for heap, matches py32f030.ld */
.section .heap, "w", %nobits
.align 3
.global __heap_base
Heap_Mem:
.space Heap_Size
__heap_limit:

/* Vector Table */
.section .vectors, "a", %progbits
.align 2
.global __Vectors
.global __Vectors_End
.global __Vectors_Size

__Vectors:
    .word __initial_sp              /* Top of Stack */
    .word Reset_Handler             /* Reset Handler */
    .word NMI_Handler               /* NMI Handler */
    .word HardFault_Handler         /* Hard Fault Handler */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word SVC_Handler               /* SVCall Handler */
    .word 0                         /* Reserved */
    .word 0                         /* Reserved */
    .word PendSV_Handler            /* PendSV Handler */
    .word SysTick_Handler           /* SysTick Handler */
    /* External Interrupts */
    .word WWDG_IRQHandler           /* 0: Window Watchdog */
    .word PVD_IRQHandler            /* 1: PVD through EXTI Line detect */
    .word RTC_IRQHandler            /* 2: RTC through EXTI Line */
    .word FLASH_IRQHandler          /* 3: FLASH */
    .word RCC_IRQHandler            /* 4: RCC */
    .word EXTI0_1_IRQHandler        /* 5: EXTI Line 0 and 1 */
    .word EXTI2_3_IRQHandler        /* 6: EXTI Line 2 and 3 */
    .word EXTI4_15_IRQHandler       /* 7: EXTI Line 4 to 15 */
    .word 0                         /* 8: Reserved */
    .word DMA1_Channel1_IRQHandler  /* 9: DMA1 Channel 1 */
    .word DMA1_Channel2_3_IRQHandler /* 10: DMA1 Channel 2 and Channel 3 */
    .word 0                         /* 11: Reserved */
    .word ADC_COMP_IRQHandler       /* 12: ADC & COMP */
    .word TIM1_BRK_UP_TRG_COM_IRQHandler /* 13: TIM1 Break, Update, Trigger and Commutation */
    .word TIM1_CC_IRQHandler        /* 14: TIM1 Capture Compare */
    .word 0                         /* 15: Reserved */
    .word TIM3_IRQHandler           /* 16: TIM3 */
    .word LPTIM1_IRQHandler         /* 17: LPTIM1 */
    .word 0                         /* 18: Reserved */
    .word TIM14_IRQHandler          /* 19: TIM14 */
    .word 0                         /* 20: Reserved */
    .word TIM16_IRQHandler          /* 21: TIM16 */
    .word TIM17_IRQHandler          /* 22: TIM17 */
    .word I2C1_IRQHandler           /* 23: I2C1 */
    .word 0                         /* 24: Reserved */
    .word SPI1_IRQHandler           /* 25: SPI1 */
    .word SPI2_IRQHandler           /* 26: SPI2 */
    .word USART1_IRQHandler         /* 27: USART1 */
    .word USART2_IRQHandler         /* 28: USART2 */
    .word 0                         /* 29: Reserved */
    .word LED_IRQHandler            /* 30: LED */
    .word 0                         /* 31: Reserved */
__Vectors_End:

.equ __Vectors_Size, __Vectors_End - __Vectors

/* Reset Handler */
.section .text.Reset_Handler, "ax", %progbits
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
    /* Copy .data section from Flash to SRAM */
    ldr r0, =__data_load_start
    ldr r1, =__data_start
    ldr r2, =__data_end
    subs r2, r2, r1
    beq 2f
1:
    ldr r3, [r0]
    str r3, [r1]
    adds r0, #4
    adds r1, #4
    subs r2, #4
    bne 1b
2:
    /* Zero .bss section */
    ldr r0, =__bss_start
    ldr r1, =__bss_end
    movs r2, #0
    subs r1, r1, r0
    beq 4f
3:
    str r2, [r0]
    adds r0, #4
    subs r1, #4
    bne 3b
4:
    /* Call SystemInit */
    ldr r0, =SystemInit
    blx r0
    /* Call main */
    ldr r0, =main
    bx r0
.size Reset_Handler, . - Reset_Handler

/* Exception Handlers */
.section .text.Default_Handler, "ax", %progbits
.weak NMI_Handler
.type NMI_Handler, %function
NMI_Handler:
    bx lr
.size NMI_Handler, . - NMI_Handler

.weak HardFault_Handler
.type HardFault_Handler, %function
HardFault_Handler:
    bx lr
.size HardFault_Handler, . - HardFault_Handler

.weak SVC_Handler
.type SVC_Handler, %function
SVC_Handler:
    bx lr
.size SVC_Handler, . - SVC_Handler

.weak PendSV_Handler
.type PendSV_Handler, %function
PendSV_Handler:
    bx lr
.size PendSV_Handler, . - PendSV_Handler

.weak SysTick_Handler
.type SysTick_Handler, %function
SysTick_Handler:
    bx lr
.size SysTick_Handler, . - SysTick_Handler

.weak WWDG_IRQHandler
.type WWDG_IRQHandler, %function
WWDG_IRQHandler:
    bx lr
.size WWDG_IRQHandler, . - WWDG_IRQHandler

.weak PVD_IRQHandler
.type PVD_IRQHandler, %function
PVD_IRQHandler:
    bx lr
.size PVD_IRQHandler, . - PVD_IRQHandler

.weak RTC_IRQHandler
.type RTC_IRQHandler, %function
RTC_IRQHandler:
    bx lr
.size RTC_IRQHandler, . - RTC_IRQHandler

.weak FLASH_IRQHandler
.type FLASH_IRQHandler, %function
FLASH_IRQHandler:
    bx lr
.size FLASH_IRQHandler, . - FLASH_IRQHandler

.weak RCC_IRQHandler
.type RCC_IRQHandler, %function
RCC_IRQHandler:
    bx lr
.size RCC_IRQHandler, . - RCC_IRQHandler

.weak EXTI0_1_IRQHandler
.type EXTI0_1_IRQHandler, %function
EXTI0_1_IRQHandler:
    bx lr
.size EXTI0_1_IRQHandler, . - EXTI0_1_IRQHandler

.weak EXTI2_3_IRQHandler
.type EXTI2_3_IRQHandler, %function
EXTI2_3_IRQHandler:
    bx lr
.size EXTI2_3_IRQHandler, . - EXTI2_3_IRQHandler

.weak EXTI4_15_IRQHandler
.type EXTI4_15_IRQHandler, %function
EXTI4_15_IRQHandler:
    bx lr
.size EXTI4_15_IRQHandler, . - EXTI4_15_IRQHandler

.weak DMA1_Channel1_IRQHandler
.type DMA1_Channel1_IRQHandler, %function
DMA1_Channel1_IRQHandler:
    bx lr
.size DMA1_Channel1_IRQHandler, . - DMA1_Channel1_IRQHandler

.weak DMA1_Channel2_3_IRQHandler
.type DMA1_Channel2_3_IRQHandler, %function
DMA1_Channel2_3_IRQHandler:
    bx lr
.size DMA1_Channel2_3_IRQHandler, . - DMA1_Channel2_3_IRQHandler

.weak ADC_COMP_IRQHandler
.type ADC_COMP_IRQHandler, %function
ADC_COMP_IRQHandler:
    bx lr
.size ADC_COMP_IRQHandler, . - ADC_COMP_IRQHandler

.weak TIM1_BRK_UP_TRG_COM_IRQHandler
.type TIM1_BRK_UP_TRG_COM_IRQHandler, %function
TIM1_BRK_UP_TRG_COM_IRQHandler:
    bx lr
.size TIM1_BRK_UP_TRG_COM_IRQHandler, . - TIM1_BRK_UP_TRG_COM_IRQHandler

.weak TIM1_CC_IRQHandler
.type TIM1_CC_IRQHandler, %function
TIM1_CC_IRQHandler:
    bx lr
.size TIM1_CC_IRQHandler, . - TIM1_CC_IRQHandler

.weak TIM3_IRQHandler
.type TIM3_IRQHandler, %function
TIM3_IRQHandler:
    bx lr
.size TIM3_IRQHandler, . - TIM3_IRQHandler

.weak LPTIM1_IRQHandler
.type LPTIM1_IRQHandler, %function
LPTIM1_IRQHandler:
    bx lr
.size LPTIM1_IRQHandler, . - LPTIM1_IRQHandler

.weak TIM14_IRQHandler
.type TIM14_IRQHandler, %function
TIM14_IRQHandler:
    bx lr
.size TIM14_IRQHandler, . - TIM14_IRQHandler

.weak TIM16_IRQHandler
.type TIM16_IRQHandler, %function
TIM16_IRQHandler:
    bx lr
.size TIM16_IRQHandler, . - TIM16_IRQHandler

.weak TIM17_IRQHandler
.type TIM17_IRQHandler, %function
TIM17_IRQHandler:
    bx lr
.size TIM17_IRQHandler, . - TIM17_IRQHandler

.weak I2C1_IRQHandler
.type I2C1_IRQHandler, %function
I2C1_IRQHandler:
    bx lr
.size I2C1_IRQHandler, . - I2C1_IRQHandler

.weak SPI1_IRQHandler
.type SPI1_IRQHandler, %function
SPI1_IRQHandler:
    bx lr
.size SPI1_IRQHandler, . - SPI1_IRQHandler

.weak SPI2_IRQHandler
.type SPI2_IRQHandler, %function
SPI2_IRQHandler:
    bx lr
.size SPI2_IRQHandler, . - SPI2_IRQHandler

.weak USART1_IRQHandler
.type USART1_IRQHandler, %function
USART1_IRQHandler:
    bx lr
.size USART1_IRQHandler, . - USART1_IRQHandler

.weak USART2_IRQHandler
.type USART2_IRQHandler, %function
USART2_IRQHandler:
    bx lr
.size USART2_IRQHandler, . - USART2_IRQHandler

.weak LED_IRQHandler
.type LED_IRQHandler, %function
LED_IRQHandler:
    bx lr
.size LED_IRQHandler, . - LED_IRQHandler

.end
