
#include "stmflash.h"



//addr:读地址，没有特殊要求？任意地址都可以读，应该
void eeprom_read_nbyte(unsigned short raddr, unsigned char* buf,  unsigned short read_len)
{
  unsigned char* t_addr;
  t_addr = (unsigned char* )(EEPROM_BASE_ADDR + raddr);
  while(read_len--)
  {
    *buf++ = *(volatile unsigned char*)t_addr++;    
  }
  
}

bool eeprom_write_nbyte(unsigned short waddr, unsigned char* buf,  unsigned short write_len)
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




