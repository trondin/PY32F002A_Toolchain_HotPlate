#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "py32f030x6.h"
#include "systick.h"


/*
pin usage:
LCD_EN - PF1
LCD_RS - PB1
LCD_D4 - PF0
LCD_D5 - PA5
LCD_D6 - PA6
LCD_D7 - PA14
*/

// LCD pin definitions
#define LCD_EN_PORT  GPIOF
#define LCD_EN_PIN   GPIO_ODR_OD1
#define LCD_RS_PORT  GPIOB
#define LCD_RS_PIN   GPIO_ODR_OD1
#define LCD_D4_PORT  GPIOF
#define LCD_D4_PIN   GPIO_ODR_OD0
#define LCD_D5_PORT  GPIOA
#define LCD_D5_PIN   GPIO_ODR_OD5
#define LCD_D6_PORT  GPIOA
#define LCD_D6_PIN   GPIO_ODR_OD6
#define LCD_D7_PORT  GPIOA
#define LCD_D7_PIN   GPIO_ODR_OD14

// LCD dimensions (configurable)
#define LCD_ROWS 4
#define LCD_COLS 10

// LCD commands
#define LCD_CLEAR       0x01
#define LCD_HOME        0x02
#define LCD_ENTRY_MODE  0x04
#define LCD_DISPLAY     0x08
#define LCD_SHIFT       0x10
#define LCD_FUNCTION    0x20
#define LCD_CGRAM       0x40
#define LCD_DDRAM       0x80

// Function prototypes
void LCD_Init(void);
void LCD_SetCursor(uint8_t col, uint8_t  row);
void LCD_WriteChar(char c);
void LCD_WriteString(const char* str);
void LCD_DefineChar(uint8_t code, const uint8_t* bitmap);
void Draw3Num(uint8_t posX, uint8_t posY, uint8_t num);

#endif