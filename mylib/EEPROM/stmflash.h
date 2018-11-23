#ifndef STMFLASH_H_
#define STMFLASH_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "platform_config.h"  
#include "stm32l1xx_flash.h"
#include "stdbool.h"  
#include "stdio.h"
#include <stdarg.h>
  
#define EEPROM_BASE_ADDR    0x08080000      //(0x08080000 ~ 0x08080FFF) 4Kbye

#define EEPROM_BYTE_SIZE	0x0FFF  
  
void eeprom_read_nbyte(unsigned short raddr, unsigned char* buf,  unsigned short read_len);

bool eeprom_write_nbyte(unsigned short waddr, unsigned char* buf,  unsigned short write_len);
  
    
  
#ifdef __cplusplus
}
#endif
#endif /* STMFLASH_H_ */