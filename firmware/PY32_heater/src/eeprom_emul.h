#ifndef EEPROM_EMUL_H
#define EEPROM_EMUL_H

#include <stdint.h>

void EEPROM_Init(uint16_t default_value);
void EEPROM_Write(uint16_t value);
uint16_t EEPROM_Read(void);
void EEPROM_Update(void);
//void EEPROM_Test(void);

#endif