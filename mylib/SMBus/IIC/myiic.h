#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "stdbool.h"


#define LTC4015_IIC 1  //����LTC4015��IIC�ӿ�ʹ��MCU��IIC1
#define CAT5140_IIC 2  //����LTC4015��IIC�ӿ�ʹ��MCU��IIC2

#if defined (USE_STM32L152_EVAL)

#define SDA_1_IN()  {GPIOB->MODER&=0XFFFF3FFF;GPIOB->MODER|=0<<14;}
#define SDA_1_OUT() {GPIOB->MODER&=0XFFFF3FFF;GPIOB->MODER|=1<<14;GPIOB->OTYPER&=0XFF7F;GPIOB->OTYPER|=0<<7;}

#define SDA_2_IN()  {GPIOB->MODER&=0XFF3FFFFF;GPIOB->MODER|=0<<22;}
#define SDA_2_OUT() {GPIOB->MODER&=0XFF3FFFFF;GPIOB->MODER|=1<<22;GPIOB->OTYPER&=0XF7FF;GPIOB->OTYPER|=0<<11;}



#define IIC_1_SCL       PBout(6) //SCL:1
#define IIC_1_SDA       PBout(7) //SDA	:1
#define READ_1_SDA      PBin(7)  //����SDA:1


#define IIC_2_SCL       PBout(10) //SCL:2
#define IIC_2_SDA       PBout(11) //SDA	:2 
#define READ_2_SDA      PBin(11)  //����SDA:2 


#else
//IO��������
#define SDA_1_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<28;}
#define SDA_1_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<28;}

#define SDA_2_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_2_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO��������	
#define IIC_1_SCL       PBout(6) //SCL:1
#define IIC_1_SDA       PBout(7) //SDA	:1
#define READ_1_SDA      PBin(7)  //����SDA:1

#define IIC_2_SCL       PBout(10) //SCL:2
#define IIC_2_SDA       PBout(11) //SDA	:2 
#define READ_2_SDA      PBin(11)  //����SDA:2 


#endif

//IIC���в�������
void IIC_Init(void);                            //��ʼ��IIC1,IIC2��IO��

void IIC_Start(unsigned char IIC_Addr);	//����IIC��ʼ�ź�
void IIC_Stop(unsigned char IIC_Addr);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(unsigned char IIC_Addr, uint8_t txd);			//IIC����һ���ֽ�
uint8_t IIC_Read_Byte(unsigned char IIC_Addr, unsigned char ack);            //IIC��ȡһ���ֽ�
uint8_t IIC_Wait_Ack(unsigned char IIC_Addr); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(unsigned char IIC_Addr);			        //IIC����ACK�ź�
void IIC_NAck(unsigned char IIC_Addr);				//IIC������ACK�ź�

bool IIC_Write_Nbytes(unsigned char IIC_NUMB, uint8_t * data_ptr, uint8_t num_bytes);
bool IIC_Read_Nbytes(unsigned char IIC_NUMB, uint8_t * data_ptr, uint8_t num_bytes);

//void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
//uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















