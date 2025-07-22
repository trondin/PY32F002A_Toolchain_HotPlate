#include "py32f030x6.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "eeprom_emul.h"
#include "systick.h"

// Flash configuration
#define EEPROM_BASE_ADDR   ((uint32_t*)0x08007F80) // Last flash page (128 bytes)
#define EEPROM_SIZE        128                     // Page size in bytes
#define EEPROM_INVALID     0xFFFF                  // Invalid marker for uint16_t
#define WORDS_PER_PAGE     (EEPROM_SIZE / 2)       // 64 half-words (2 bytes each)
#define BUFFER_SIZE        (WORDS_PER_PAGE + 4)    // 68 half-words to workaround bug

// Static variables
static uint16_t eeprom_value = 0;           // Current EEPROM value
static uint16_t previous_eeprom_value = 0;           // Current EEPROM value
static volatile uint8_t write_pending = 0;  // Write pending flag
static uint8_t current_write_addr = 0;      // Current write position
static uint8_t erase_required = 0;          // Erase required flag

__attribute__((aligned(4))) static uint16_t eeprom_buffer[BUFFER_SIZE]; // 68 words to workaround bug



// Initialize EEPROM
void EEPROM_Init(uint16_t default_value)
{
  // Copy flash to buffer
  uint32_t* src = EEPROM_BASE_ADDR;
  uint32_t* dst = (uint32_t*)eeprom_buffer;  
  uint8_t i = 32;
  while (i--) *dst++ = *src++;

  // Find first valid value
  for (uint8_t i = WORDS_PER_PAGE - 1; i != 255; i--) 
  {
    if (eeprom_buffer[i] != EEPROM_INVALID)
    {
      eeprom_value = eeprom_buffer[i];
      previous_eeprom_value = eeprom_value;
      current_write_addr = i+1;
      return;
    }
  }
  // No valid value found, use default
  eeprom_value = default_value;
  previous_eeprom_value = eeprom_value;  
  eeprom_buffer[0] = default_value;
  current_write_addr = 1;
  write_pending = 1;
  erase_required = 0;
}

// Write value to EEPROM
void EEPROM_Write_Upd(void)
{
  if (previous_eeprom_value != eeprom_value) 
  {      
    // Check if buffer is full
    if (current_write_addr >= WORDS_PER_PAGE)
    {
      current_write_addr = 0;
      erase_required = 1;
      uint32_t* dst = (uint32_t*)eeprom_buffer;  
      uint8_t i = 32;
      while (i--) *dst++ = 0xFFFFFFFF;       
    }
        
    eeprom_buffer[current_write_addr] = eeprom_value;
    previous_eeprom_value  = eeprom_value;
    current_write_addr++;
    write_pending = 1;
  }
}

// Read current EEPROM value
uint16_t EEPROM_Read(void)
{
  return eeprom_value;
}

void EEPROM_Write(uint16_t value)
{
  eeprom_value = value;
}

// Flash write core function (runs from RAM) - restored from original
__attribute__((aligned(4))) static uint8_t ram_code[128];
__attribute__((aligned(4))) static void flash_write_core(void) 
{
  uint32_t *dst = EEPROM_BASE_ADDR; 
  uint32_t *src = (uint32_t *)eeprom_buffer; 
  uint8_t i = 31;
  while (i--) *dst++ = *src++;      // Write 31 words
  FLASH->CR |= FLASH_CR_PGSTRT;     // Start programming
  *dst = *src;                      // Write 32nd word
  while (FLASH->SR & FLASH_SR_BSY); // Wait for completion
  __ISB(); __DSB();
}

// Flash erase core function (runs from RAM) - restored from original
__attribute__((aligned(4))) static void flash_erase_core(void) 
{
  FLASH->CR |= FLASH_CR_PER | FLASH_CR_EOPIE; // Set PER and EOPIE
  *(uint32_t *)EEPROM_BASE_ADDR = 0xFFFFFFFF; // Write arbitrary data
  FLASH->CR |= FLASH_CR_PGSTRT;              // Start operation
  while (FLASH->SR & FLASH_SR_BSY);          // Wait for completion
  __ISB(); __DSB();
}

typedef void (*FlashFunc)(void);


void EEPROM_erase(void)
{
  uint32_t *src = (uint32_t *)((uint32_t)flash_erase_core & ~0x1); // Thumb
  uint32_t *dst = (uint32_t *)ram_code; 
  uint16_t i = sizeof(ram_code)/4;
  while (i--) *dst++ = *src++;    
  FlashFunc ram_flash_erase = (FlashFunc)((uint32_t)ram_code | 0x1);  

  // Erase page - restored from original
  __disable_irq();
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  __ISB(); __DSB();
  ram_flash_erase();

  while (!(FLASH->SR & FLASH_SR_EOP));
  FLASH->SR |= FLASH_SR_EOP; // Clear EOP flag
  FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_EOPIE);
  FLASH->CR |= FLASH_CR_LOCK;
  __enable_irq();  
  erase_required = 0;
}


// Update flash with buffer contents
void EEPROM_Update(void)
{
  EEPROM_Write_Upd();

  if (!write_pending) return;

  if (erase_required) EEPROM_erase();

  // Copy write function to RAM - restored from original
  uint32_t *src = (uint32_t *)((uint32_t)flash_write_core & ~0x1); // Thumb
  uint32_t *dst = (uint32_t *)ram_code; 
  uint16_t i = sizeof(ram_code)/4;
  while (i--) *dst++ = *src++;    


  // Write buffer to flash
  __disable_irq();
  while (FLASH->SR & FLASH_SR_BSY);
  FLASH->KEYR = 0x45670123;
  FLASH->KEYR = 0xCDEF89AB;
  FLASH->CR |= FLASH_CR_PG | FLASH_CR_EOPIE;  
  __ISB(); __DSB();

  FlashFunc ram_flash_write = (FlashFunc)((uint32_t)ram_code | 0x1);  
  ram_flash_write();

  while (!(FLASH->SR & FLASH_SR_EOP));
  FLASH->SR |= FLASH_SR_EOP; // Clear EOP flag
  FLASH->CR &= ~(FLASH_CR_PG | FLASH_CR_EOPIE);
  FLASH->CR |= FLASH_CR_LOCK;
  __enable_irq();  
  write_pending = 0;
}

#if(0)
// Test function for EEPROM emulator with printf output
void EEPROM_Test(void)
{
    uint16_t test_value = 0x1234; // Test value for initialization
    uint16_t read_value;

    printf("Starting EEPROM test...\n");
    // 1. Initialize EEPROM
    printf("Initializing EEPROM with value 0x%04X\n", test_value);
    EEPROM_Init(test_value);
    read_value = EEPROM_Read();
    printf("Read value after initialization: 0x%04X %s\n", read_value,
           read_value == test_value ? "(OK)" : "(FAIL)");

    // 2. Test write and read
    test_value = 0x5678;
    printf("Writing value 0x%04X\n", test_value);
    EEPROM_Write(test_value);
    read_value = EEPROM_Read();
    printf("Read value after write: 0x%04X %s\n", read_value,
           read_value == test_value ? "(OK)" : "(FAIL)");

    // 3. Update Flash
    printf("Updating Flash...\n");
    EEPROM_Update();
    read_value = EEPROM_Read();
    printf("Read value after Flash update: 0x%04X %s\n", read_value,
           read_value == test_value ? "(OK)" : "(FAIL)");

    // 4. Test buffer overflow (emulate page erase)
    printf("Filling buffer to trigger erase...\n");
    for (uint8_t i = 0; i < 62; i++) { // 62 to reach WORDS_PER_PAGE limit
        EEPROM_Write(test_value + i);
    }
    read_value = EEPROM_Read();
    printf("Read value after filling buffer: 0x%04X %s\n", read_value,
           read_value == test_value + 61 ? "(OK)" : "(FAIL)");

    // 5. Write to trigger erase
    test_value = 0xABCD;
    printf("Writing value 0x%04X to trigger erase\n", test_value);
    EEPROM_Write(test_value);
    printf("Updating Flash (should erase page)...\n");
    EEPROM_Update();
    read_value = EEPROM_Read();
    printf("Read value after erase: 0x%04X %s\n", read_value,
           read_value == test_value ? "(OK)" : "(FAIL)");

    // 6. Final write and read test
    test_value = 0xDEAD;
    printf("Writing final value 0x%04X\n", test_value);
    EEPROM_Write(test_value);
    printf("Updating Flash for final test...\n");
    EEPROM_Update();
    read_value = EEPROM_Read();
    printf("Read value after final write: 0x%04X %s\n", read_value,
           read_value == test_value ? "(OK)" : "(FAIL)");

    printf("EEPROM test completed.\n");
}
#endif