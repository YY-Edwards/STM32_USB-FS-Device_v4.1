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
  
//void eeprom_read_nbyte(unsigned short raddr, unsigned char* buf,  unsigned short read_len);
//bool eeprom_write_nbyte(unsigned short waddr, unsigned char* buf,  unsigned short write_len);
void read_charger_configuration(void*p, unsigned short length);
bool save_charger_configuration(void*p, unsigned short length);
bool save_current_charger_qcount(unsigned short qcount);
void read_stored_qcount(void*p);
#ifdef __cplusplus
}
#endif
#endif /* STMFLASH_H_ */