#include "lcd.h"

// Pin manipulation macros
#define SET_PIN(port, pin) ((port)->ODR |= (pin))
#define CLEAR_PIN(port, pin) ((port)->ODR &= ~(pin))


const uint8_t	FontShape[][8]	=	{
//*	LT[8] =
  0b00111,
  0b01111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111, 
//*	UB[8] =
  0b11111,
  0b11111,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
//*	RT[8] =
  0b11100,
  0b11110,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
//*	LL[8] =
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b01111,
  0b00111,
//*	LB[8] =
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111,
//*	LR[8] =
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11111,
  0b11110,
  0b11100,
//*	UMB[8] =
  0b11111,
  0b11111,
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
//*	LMB[8] =
  0b11111,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b11111,
  0b11111,
  0b11111
};

const uint8_t	FontTable[][6]	=	{
	0,	1,	2,	3,	4,	5,		//	0
	32,	3,	32,	32,	2,	32,		//	1
	6,	6,	2,	3,	7,	7,		//	2
	6,	6,	2,	7,	7,	5,		//	3
	3,	4,	2,	32,	32,	5,		//	4
	0,	6,	6,	7,	7,	5,		//	5
	0,	6,	6,	3,	7,	5,		//	6
	1,	1,	5,	32,	0,	32,		//	7
	0,	6,	2,	3,	7,	5,		//	8
	0,	6,	2,	7,	7,	5,		//	9
	};


void delay_us(uint32_t microseconds) {
    microseconds *= 48;  // 48 clock cycles per microsecond
    
    __asm volatile (
        "mov r1, %0      \n"  // Load counter into r1
        "loop:           \n"
        "sub r1, #1      \n"  // Decrement counter (Thumb-compatible instruction)
        "bne loop        \n"  // Repeat until zero
        : : "r" (microseconds) : "r1"
    );
}

void BigFontInit(void)
{
  // create a new characters
  for(uint8_t i=0; i<8; i++) LCD_DefineChar(i, FontShape[i]); 
}

void DrawNum(uint8_t posX, uint8_t posY, uint8_t num)
{
  uint8_t X;
  if (num>9) num=0;
  uint8_t Y=posY;
  uint8_t shape=0;  
  for (uint8_t y=0; y<2; y++)  
  {
    X=posX;     
    LCD_SetCursor(X, Y);    
    for (uint8_t x=0; x<3; x++) LCD_WriteChar(FontTable[num][shape++]);
    Y++;
  }
}


void Draw3Num(uint8_t posX, uint8_t posY, uint8_t num)
{
  //if(num>999) num=999;

  LCD_SetCursor(0, posY);    
  for(uint8_t x=0; x<10; x++) LCD_WriteChar(' ');
  LCD_SetCursor(0, posY+1);    
  for(uint8_t x=0; x<10; x++) LCD_WriteChar(' ');  

  DrawNum(posX+6, posY, num % 10);
  num /= 10;
  if (num>0)
  {
    DrawNum(posX+3, posY, num % 10);
    num /= 10;    
    if (num>0) DrawNum(posX, posY, num);
  }
}

// Internal functions
static void LCD_PulseEnable(void)
{
    SET_PIN(LCD_EN_PORT, LCD_EN_PIN);
    delay_us(200);
    CLEAR_PIN(LCD_EN_PORT, LCD_EN_PIN);
    delay_us(200);
}

static void LCD_WriteNibble(uint8_t nibble)
{
    CLEAR_PIN(LCD_D4_PORT, LCD_D4_PIN);
    CLEAR_PIN(LCD_D5_PORT, LCD_D5_PIN);
    CLEAR_PIN(LCD_D6_PORT, LCD_D6_PIN);
    CLEAR_PIN(LCD_D7_PORT, LCD_D7_PIN);
    
    if (nibble & 0x01) SET_PIN(LCD_D4_PORT, LCD_D4_PIN);
    if (nibble & 0x02) SET_PIN(LCD_D5_PORT, LCD_D5_PIN);
    if (nibble & 0x04) SET_PIN(LCD_D6_PORT, LCD_D6_PIN);
    if (nibble & 0x08) SET_PIN(LCD_D7_PORT, LCD_D7_PIN);
    
    LCD_PulseEnable();
}

static void LCD_WriteByte(uint8_t data, uint8_t rs)
{
  if (rs) SET_PIN(LCD_RS_PORT, LCD_RS_PIN);
  else    CLEAR_PIN(LCD_RS_PORT, LCD_RS_PIN);
    
  LCD_WriteNibble(data >> 4);   // High nibble
  delay_us(100); 
  LCD_WriteNibble(data & 0x0F); // Low nibble
  delay_us(100);
}

// Calculate line addresses based on LCD dimensions
static uint8_t LCD_GetLineAddress(uint8_t row)
{
  // For 4-row displays (4x10)    
  return (row == 0) ? 0x00 : (row == 1) ? 0x40 : (row == 2) ? 0x0A : (0x4A);    
}

static void init_lcd_pins(void) 
{
    // Enable clocks for GPIOA, GPIOB, GPIOF
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOFEN;
    
    // PF1 (LCD_EN)
    GPIOF->MODER = (GPIOF->MODER & ~GPIO_MODER_MODE1) | GPIO_MODER_MODE1_0; // Output mode
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT1; // Push-pull
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL1; // Clear alternate function
    
    // PF0 (LCD_D4)
    GPIOF->MODER = (GPIOF->MODER & ~GPIO_MODER_MODE0) | GPIO_MODER_MODE0_0;
    GPIOF->OTYPER &= ~GPIO_OTYPER_OT0;
    GPIOF->AFR[0] &= ~GPIO_AFRL_AFSEL0;
    
    // PA5 (LCD_D5)
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE5) | GPIO_MODER_MODE5_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5;
    
    // PA6 (LCD_D6)
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE6) | GPIO_MODER_MODE6_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT6;
    GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6;
    
    // PA14 (LCD_D7)
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODE14) | GPIO_MODER_MODE14_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT14;
    GPIOA->AFR[1] &= ~GPIO_AFRH_AFSEL14;
    
    // PB1 (LCD_RS)
    GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODE1) | GPIO_MODER_MODE1_0;
    GPIOB->OTYPER &= ~GPIO_OTYPER_OT1;
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL1;
}

void LCD_Init(void)
{
    init_lcd_pins();
    
    // Initial delay after power-on
    delay_ms(50);
    
    // Set 4-bit mode
    CLEAR_PIN(LCD_RS_PORT, LCD_RS_PIN);
    LCD_WriteNibble(0x03);
    delay_us(200);
    LCD_WriteNibble(0x03);
    delay_us(200);
    LCD_WriteNibble(0x03);
    delay_us(200);
    LCD_WriteNibble(0x02);
    delay_us(200);
    // Function set: 4-bit mode, configure lines based on LCD_ROWS
    uint8_t function_set = LCD_FUNCTION  | 0x08 | 0x02; // 4-bit, 5x8 font, more than 1 row
    LCD_WriteByte(function_set, 0);
    delay_us(200);
    
    // Display on, cursor off, blink off
    LCD_WriteByte(LCD_DISPLAY | 0x04, 0);
    delay_us(200);
    
    // Clear display
    LCD_WriteByte(LCD_CLEAR, 0);
    delay_us(200);
    
    // Entry mode: increment cursor, no shift
    LCD_WriteByte(LCD_ENTRY_MODE | 0x02, 0);
    delay_us(200);

    BigFontInit();
}

void LCD_SetCursor(uint8_t col, uint8_t  row)
{
    if (row >= LCD_ROWS || col >= LCD_COLS) return;
    LCD_WriteByte(LCD_DDRAM | (LCD_GetLineAddress(row) + col), 0);
    delay_us(100);
}

void LCD_WriteChar(char c)
{
    LCD_WriteByte(c, 1);
    delay_us(100);
}

void LCD_WriteString(const char* str)
{
  while (*str) LCD_WriteChar(*str++);
}

void LCD_DefineChar(uint8_t code, const uint8_t* bitmap)
{
    if (code > 7) return; // Only 8 custom characters supported   
    LCD_WriteByte(LCD_CGRAM | (code << 3), 0);
    for (uint8_t i = 0; i < 8; i++)  LCD_WriteByte(bitmap[i], 1);
    // Return to DDRAM
    LCD_WriteByte(LCD_DDRAM, 0);
    delay_us(100);
}
