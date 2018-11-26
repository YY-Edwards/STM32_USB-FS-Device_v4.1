
#include "stmflash.h"



//addr:����ַ��û������Ҫ�������ַ�����Զ���Ӧ��
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
  //�����Ƿ���Ҫ�ȹر��жϣ���������CPU�쳣
  FLASH_Status status_ = FLASH_COMPLETE;
  
  unsigned int t_addr;
  t_addr = EEPROM_BASE_ADDR + waddr;
  if(t_addr >=(EEPROM_BASE_ADDR+EEPROM_BYTE_SIZE))return false;
  
  DATA_EEPROM_Unlock();

  //����д֮ǰ���ڲ�Ϊ0xFF����������д�Ƿ�ɹ�����˼��д֮ǰ�Ƿ���Ҫ����������
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




