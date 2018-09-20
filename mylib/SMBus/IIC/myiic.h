#ifndef __MYIIC_H
#define __MYIIC_H
#include "sys.h"
#include "stdbool.h"


#define LTC4015_IIC 1  //定义LTC4015的IIC接口使用MCU的IIC1
#define CAT5140_IIC 2  //定义LTC4015的IIC接口使用MCU的IIC2


//IO方向设置
#define SDA_1_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<12;}
#define SDA_1_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<12;}

#define SDA_2_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define SDA_2_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}


//IO操作函数	
#define IIC_1_SCL       PBout(6) //SCL:1
#define IIC_1_SDA       PBout(7) //SDA	:1
#define READ_1_SDA      PBin(7)  //输入SDA:1

#define IIC_2_SCL       PBout(10) //SCL:2
#define IIC_2_SDA       PBout(11) //SDA	:2 
#define READ_2_SDA      PBin(11)  //输入SDA:2 

//IIC所有操作函数
void IIC_Init(void);                            //初始化IIC1,IIC2的IO口

void IIC_Start(unsigned char IIC_Addr);	//发送IIC开始信号
void IIC_Stop(unsigned char IIC_Addr);	  			//发送IIC停止信号
void IIC_Send_Byte(unsigned char IIC_Addr, u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char IIC_Addr, unsigned char ack);            //IIC读取一个字节
u8 IIC_Wait_Ack(unsigned char IIC_Addr); 				//IIC等待ACK信号
void IIC_Ack(unsigned char IIC_Addr);			        //IIC发送ACK信号
void IIC_NAck(unsigned char IIC_Addr);				//IIC不发送ACK信号

bool IIC_Write_Nbytes(unsigned char IIC_NUMB, u8 * data_ptr, u8 num_bytes);
bool IIC_Read_Nbytes(unsigned char IIC_NUMB, u8 * data_ptr, u8 num_bytes);

//void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
//u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















