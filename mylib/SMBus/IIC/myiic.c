#include "myiic.h"
#include "delay.h"

 
//初始化IIC
void IIC_Init(void)
{					     
      GPIO_InitTypeDef GPIO_InitStructure;
#if defined(STM32L1XX_MD)
      
      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
      
//      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11;
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_3|GPIO_Pin_4;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
      GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;//GPIO_OType_PP; /* Push-pull or open drain */
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* None, Pull-up or pull-down */
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz; /* 400 KHz, 2, 10 or 40MHz */
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      
      //GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11); 	//PB6,PB7, PB10,PB11 输出高
       
      GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_3|GPIO_Pin_4); 	//PB6,PB7, PB3,PB4 输出高
   
      
      SDA_1_IN();
      SDA_1_OUT();
      SDA_1_IN();
      SDA_1_OUT();

      SDA_2_IN();
      SDA_2_OUT();
      SDA_2_IN();
      SDA_2_OUT();
#else      
      
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	
         
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11); 	//PB6,PB7, PB10,PB11 输出高
      
#endif
}
//产生IIC起始信号
void IIC_Start(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	SDA_1_OUT();     //sda1线输出
	IIC_1_SDA=1;	  	  
	IIC_1_SCL=1;
	delay_us(7);
 	IIC_1_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(7);
	IIC_1_SCL=0;//钳住I2C1总线，准备发送或接收数据 
  }
  else
  {
        SDA_2_OUT();     //sda2线输出
	IIC_2_SDA=1;	  	  
	IIC_2_SCL=1;
	delay_us(7);
 	IIC_2_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(7);
	IIC_2_SCL=0;//钳住I2C2总线，准备发送或接收数据    
  }
}	  
//产生IIC停止信号
void IIC_Stop(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	SDA_1_OUT();//sda1线输出
	IIC_1_SCL=0;
	IIC_1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(7);
	IIC_1_SCL=1; 
        delay_us(7);
	IIC_1_SDA=1;//发送I2C总线结束信号
	delay_us(7);	
  }
  else
  {
	SDA_2_OUT();//sda2线输出
	IIC_2_SCL=0;
	IIC_2_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(7);
	IIC_2_SCL=1; 
        delay_us(7);
	IIC_2_SDA=1;//发送I2C总线结束信号
	delay_us(7);	
  }

}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(unsigned char IIC_Addr)
{
  uint8_t ucErrTime;
  
   if(IIC_Addr == LTC4015_IIC)
  {
      ucErrTime=0;
      SDA_1_IN();      //SDA1设置为输入  
      IIC_1_SDA=1;delay_us(1);	   
      IIC_1_SCL=1;delay_us(1);	 
      while(READ_1_SDA)
      {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop(LTC4015_IIC);
            return 1;
        }
      }
      IIC_1_SCL=0;//应答成功，时钟输出0    
      return 0;  
  }
  else
  {
    ucErrTime=0;
    SDA_2_IN();      //SDA2设置为输入  
    IIC_2_SDA=1;delay_us(1);	   
    IIC_2_SCL=1;delay_us(1);	 
    while(READ_2_SDA)
    {
      ucErrTime++;
      if(ucErrTime>250)
      {
          IIC_Stop(CAT5140_IIC);
          return 1;
      }
    }
    IIC_2_SCL=0;//应答成功，时钟输出0 	   
    return 0;   
  }
} 
//产生ACK应答
void IIC_Ack(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	IIC_1_SCL=0;
	SDA_1_OUT();
	IIC_1_SDA=0;
	delay_us(2);
	IIC_1_SCL=1;
	delay_us(5);
	IIC_1_SCL=0;
  }
  else
  {
        IIC_2_SCL=0;
	SDA_2_OUT();
	IIC_2_SDA=0;
	delay_us(2);
	IIC_2_SCL=1;
	delay_us(5);
	IIC_2_SCL=0;
  }
}
//不产生ACK应答		    
void IIC_NAck(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	IIC_1_SCL=0;
	SDA_1_OUT();
	IIC_1_SDA=1;
	delay_us(2);
	IIC_1_SCL=1;
	delay_us(5);
	IIC_1_SCL=0;
  }
  else
  {
        IIC_2_SCL=0;
	SDA_2_OUT();
	IIC_2_SDA=1;
	delay_us(2);
	IIC_2_SCL=1;
	delay_us(5);
	IIC_2_SCL=0;
  
  }
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(unsigned char IIC_Addr, uint8_t txd)
{                        
  uint8_t t;  
 if(IIC_Addr == LTC4015_IIC)
  {
    SDA_1_OUT(); 	    
    IIC_1_SCL=0;//拉低时钟开始数据传输
    delay_us(2);
    for(t=0;t<8;t++)//从高位开始一位一位地传送
    {              
        //IIC_1_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)//当前的最高位为1时
            IIC_1_SDA=1;
        else
            IIC_1_SDA=0;
        
        txd<<=1; //数据左移一位	  
        
        //开始发送数据
        delay_us(2);   //对TEA5767这三个延时都是必须的
        IIC_1_SCL=1;
        delay_us(2); 
        
        //上一个发送完毕，为下一个数据准备发送
        IIC_1_SCL=0;	
        delay_us(2);
    }	
  }
 else
 {
    SDA_2_OUT(); 	    
    IIC_2_SCL=0;//拉低时钟开始数据传输
    delay_us(2);
    for(t=0;t<8;t++)
    {              
        //IIC_2_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)
            IIC_2_SDA=1;
        else
            IIC_2_SDA=0;
        
        txd<<=1; 
        
        delay_us(2);   //对TEA5767这三个延时都是必须的
        IIC_2_SCL=1;
        delay_us(2); 
        
        IIC_2_SCL=0;	
        delay_us(2);
    }	
 }
} 	  

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(unsigned char IIC_Addr, unsigned char ack)
{
  unsigned char i,receive=0;
    
  if(IIC_Addr == LTC4015_IIC)
  { 
    
    SDA_1_IN();//SDA1设置为输入
    for(i=0;i<8;i++ )
    {
      //数据准备
      IIC_1_SCL=0; 
      delay_us(2);
      
      IIC_1_SCL=1;//主机开始读
      
      receive<<=1;
      if(READ_1_SDA)receive++; 
      
      delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck(LTC4015_IIC);//发送nACK
    else
        IIC_Ack(LTC4015_IIC); //发送ACK   
    return receive;
  }
  else
  {
    SDA_2_IN();//SDA2设置为输入
    for(i=0;i<8;i++ )
    {
      IIC_2_SCL=0; 
      delay_us(2);
      IIC_2_SCL=1;

      receive<<=1;
      if(READ_2_SDA)receive++; 
      
      delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck(CAT5140_IIC);//发送nACK
    else
        IIC_Ack(CAT5140_IIC); //发送ACK   
    return receive;
  
  
  }
}



bool IIC_Write_Nbytes(unsigned char IIC_NUMB, uint8_t * data_ptr, uint8_t num_bytes)
{
  bool res =true;
  uint8_t t;
  for(t=0; t<num_bytes; t++)
  {
    IIC_Send_Byte(IIC_NUMB, *data_ptr);
    
    if(IIC_Wait_Ack(IIC_NUMB) == 0)
    {
      res &= true;
    }
    else
    {
      res = false;
    }
  
    data_ptr++;
  }
  
  return res;
}
bool IIC_Read_Nbytes(unsigned char IIC_NUMB, uint8_t * data_ptr, uint8_t num_bytes)
{
  bool res =true;
  bool test_r= true;
  uint8_t t;
  for(t=0; t<num_bytes; t++)
  {
    if(t != (num_bytes -1))
    {
      *data_ptr = IIC_Read_Byte(IIC_NUMB, 1);//SEND ACK
    }
    else//最后一个
    {
      *data_ptr = IIC_Read_Byte(IIC_NUMB, 0);//SEND NACK
    }
    
    data_ptr++;//地址递增
    
//    if(IIC_Wait_Ack(IIC_NUMB) == 0)
//    {
//      res &= true;
//    }
//    else
//    {
//      test_r = false;
//      //res = false;
//    }
  }
  
  return res;

}


























