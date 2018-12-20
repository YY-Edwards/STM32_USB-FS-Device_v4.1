
#include "stmflash.h"

#define EEPROM_BASE_ADDR                0x08080000      //(0x08080000 ~ 0x08080FFF) 4Kbye

#define GLOBAL_CONFIG_START_ADDR        0
#define GLOBAL_CONFIG_OFFSET            0x200//512bytes

#define BAT_QCOUNT_START_ADDR         GLOBAL_CONFIG_OFFSET
#define BAT_QCOUNT_OFFSET             0x02//2bytes:0~65535


#define EEPROM_BYTE_SIZE	0x0FFF



//addr:读地址，没有特殊要求？任意地址都可以读，应该
static void eeprom_read_nbyte(unsigned short raddr, unsigned char* buf,  unsigned short read_len)
{
  unsigned char* t_addr;
  t_addr = (unsigned char* )(EEPROM_BASE_ADDR + raddr);
  while(read_len--)
  {
    *buf++ = *(volatile unsigned char*)t_addr++;    
  }
  
}

static bool eeprom_write_nbyte(unsigned short waddr, unsigned char* buf,  unsigned short write_len)
{
  //测试是否需要先关闭中断，以免引起CPU异常
  FLASH_Status status_ = FLASH_COMPLETE;
  
  unsigned int t_addr;
  t_addr = EEPROM_BASE_ADDR + waddr;
  if(t_addr >=(EEPROM_BASE_ADDR+EEPROM_BYTE_SIZE))return false;
  
  DATA_EEPROM_Unlock();

  //测试写之前，在不为0xFF的数据上重写是否成功，意思是写之前是否需要擦除操作。
  while((write_len--) && (status_ == FLASH_COMPLETE))
  {
    status_ = DATA_EEPROM_ProgramByte(t_addr, *buf);
    t_addr++;
    buf++;
  }
   

  DATA_EEPROM_Lock();
  
  if(status_ != FLASH_COMPLETE)
    return false;
  else
    return true;

}

void read_charger_configuration(void*p, unsigned short length)
{
  if(length<=GLOBAL_CONFIG_OFFSET)
  eeprom_read_nbyte(GLOBAL_CONFIG_START_ADDR, (unsigned char* )p, length);
  
}

bool save_charger_configuration(void*p, unsigned short length)
{
  bool ret = false;
  if(length<=GLOBAL_CONFIG_OFFSET)
    ret = eeprom_write_nbyte(GLOBAL_CONFIG_START_ADDR, (unsigned char *)p, length);
  return ret;
}


bool save_current_charger_qcount(unsigned short qcount)
{
  //定时缓存电池当前的实际库伦值
  //电池容量记忆功能：换了电池就不准了。
  unsigned short value = qcount;
  bool ret = false;
  ret = eeprom_write_nbyte(BAT_QCOUNT_START_ADDR, (unsigned char *)&value, BAT_QCOUNT_OFFSET);
  return ret;
  
}


void read_stored_qcount(void*p)
{
  if(p!=NULL)
  eeprom_read_nbyte(BAT_QCOUNT_START_ADDR, (unsigned char* )p, BAT_QCOUNT_OFFSET);
  
}





