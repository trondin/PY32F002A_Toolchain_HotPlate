#include "uart.h"
#include "py32f030x6.h"

#define UART USART1

#define TX_BUF_SIZE 64
#define RX_BUF_SIZE 64

static volatile uint8_t tx_buf[TX_BUF_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;

static volatile uint8_t rx_buf[RX_BUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

void uart_init(void)
{ 
  RCC->APBENR2 |= RCC_APBENR2_USART1EN; // Enable USART1 clock
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN; // Enable GPIOA clock   
  USART1->CR1 = 0; 

  // Configure pins PA2 (TX) and PA3 (RX)
  GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3); // Reset mode
  GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1); // Alternate function
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3); // Reset AF 
  GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL2_Pos) | (1 << GPIO_AFRL_AFSEL3_Pos); // AF1 for USART1

  // Configure USART1: 115200 baud, 8 bits, 1 stop bit, no parity
  //USART1->BRR = 44287000 / 115200; // Divider for 115200 baud
  USART1->BRR = 47037000 / 115200; // Divider for 115200 baud
  USART1->CR1 = USART_CR1_RE | USART_CR1_TE | USART_CR1_UE | USART_CR1_RXNEIE | USART_CR1_TXEIE; // Enable receiver, transmitter, USART, and interrupts
  NVIC_EnableIRQ(USART1_IRQn); // Enable USART1 interrupt
}

void uart_putc(char c) 
{
  uint8_t next_head = (tx_head + 1) % TX_BUF_SIZE;
  while (next_head == tx_tail); // Buffer full
  tx_buf[tx_head] = c;
  tx_head = next_head;
  USART1->CR1 |= USART_CR1_TXEIE; 
}

void uart_write(const char *s) 
{
  while (*s) uart_putc(*s++);
}

int uart_tx_busy(void) 
{
  return (tx_head != tx_tail) || !(USART1->SR & USART_SR_TC);
}

int uart_getc(char *c) 
{
  if (rx_head == rx_tail) return 0;
  *c = rx_buf[rx_tail];
  rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
  return 1;
}

void USART1_IRQHandler(void) 
{
  if (USART1->SR & USART_SR_RXNE) 
  {
    uint8_t data = USART1->DR;
    uint8_t next_head = (rx_head + 1) % RX_BUF_SIZE;
    if (next_head != rx_tail) 
    {
      rx_buf[rx_head] = data;
      rx_head = next_head;
    }
    // Otherwise: overflow — character is lost
  }

  if (USART1->SR & USART_SR_TXE) 
  {
    if (tx_head == tx_tail) USART1->CR1 &= ~USART_CR1_TXEIE;
    else
    {
      USART1->DR = tx_buf[tx_tail];
      tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
    }
  }
}
