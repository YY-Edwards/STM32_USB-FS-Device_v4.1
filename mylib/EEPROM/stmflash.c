
#include "stmflash.h"

#define EEPROM_BASE_ADDR                0x08080000      //(0x08080000 ~ 0x08080FFF) 4Kbye

#define GLOBAL_CONFIG_START_ADDR        0
#define GLOBAL_CONFIG_OFFSET            0x200//512bytes

#define BAT_QCOUNT_START_ADDR         GLOBAL_CONFIG_OFFSET
#define BAT_QCOUNT_OFFSET             0x02//2bytes:0~65535


#define EEPROM_BYTE_SIZE	0x0FFF



//addr:����ַ��û������Ҫ�������ַ�����Զ���Ӧ��
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
  //��ʱ�����ص�ǰ��ʵ�ʿ���ֵ
  //����������书�ܣ����˵�ؾͲ�׼�ˡ�
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





