#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "stdbool.h"


#define LTC4015_IIC 1  //定义LTC4015的IIC接口使用MCU的IIC1
#define CAT5140_IIC 2  //定义LTC4015的IIC接口使用MCU的IIC2

#if defined (USE_STM32L152_EVAL)

#define SDA_1_IN()  {GPIOB->MODER&=0XFFFF3FFF;GPIOB->MODER|=0<<14;}
#define SDA_1_OUT() {GPIOB->MODER&=0XFFFF3FFF;GPIOB->MODER|=1<<14;GPIOB->OTYPER&=0XFF7F;GPIOB->OTYPER|=0<<7;}

#define SDA_2_IN()  {GPIOB->MODER&=0XFF3FFFFF;GPIOB->MODER|=0<<22;}
#define SDA_2_OUT() {GPIOB->MODER&=0XFF3FFFFF;GPIOB->MODER|=1<<22;GPIOB->OTYPER&=0XF7FF;GPIOB->OTYPER|=0<<11;}



#define IIC_1_SCL       PBout(6) //SCL:1
#define IIC_1_SDA       PBout(7) //SDA	:1
#define READ_1_SDA      PBin(7)  //输入SDA:1


#define IIC_2_SCL       PBout(10) //SCL:2
#define IIC_2_SDA       PBout(11) //SDA	:2 
#define READ_2_SDA      PBin(11)  //输入SDA:2 


#else
//IO方向设置
#define SDA_1_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<28;}
#define SDA_1_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<28;}

#define SDA_2_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_2_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO操作函数	
#define IIC_1_SCL       PBout(6) //SCL:1
#define IIC_1_SDA       PBout(7) //SDA	:1
#define READ_1_SDA      PBin(7)  //输入SDA:1

#define IIC_2_SCL       PBout(10) //SCL:2
#define IIC_2_SDA       PBout(11) //SDA	:2 
#define READ_2_SDA      PBin(11)  //输入SDA:2 


#endif

//IIC所有操作函数
void IIC_Init(void);                            //初始化IIC1,IIC2的IO口

void IIC_Start(unsigned char IIC_Addr);	//发送IIC开始信号
void IIC_Stop(unsigned char IIC_Addr);	  			//发送IIC停止信号
void IIC_Send_Byte(unsigned char IIC_Addr, uint8_t txd);			//IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char IIC_Addr, unsigned char ack);            //IIC读取一个字节
uint8_t IIC_Wait_Ack(unsigned char IIC_Addr); 				//IIC等待ACK信号
void IIC_Ack(unsigned char IIC_Addr);			        //IIC发送ACK信号
void IIC_NAck(unsigned char IIC_Addr);				//IIC不发送ACK信号

bool IIC_Write_Nbytes(unsigned char IIC_NUMB, uint8_t * data_ptr, uint8_t num_bytes);
bool IIC_Read_Nbytes(unsigned char IIC_NUMB, uint8_t * data_ptr, uint8_t num_bytes);

//void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
//uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















