#include "myiic.h"
#include "delay.h"

 
//��ʼ��IIC
void IIC_Init(void)
{					     
      GPIO_InitTypeDef GPIO_InitStructure;
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	
         
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &GPIO_InitStructure);
      GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_11); 	//PB6,PB7, PB10,PB11 �����
}
//����IIC��ʼ�ź�
void IIC_Start(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	SDA_1_OUT();     //sda1�����
	IIC_1_SDA=1;	  	  
	IIC_1_SCL=1;
	delay_us(4);
 	IIC_1_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_1_SCL=0;//ǯסI2C1���ߣ�׼�����ͻ�������� 
  }
  else
  {
        SDA_2_OUT();     //sda2�����
	IIC_2_SDA=1;	  	  
	IIC_2_SCL=1;
	delay_us(4);
 	IIC_2_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_2_SCL=0;//ǯסI2C2���ߣ�׼�����ͻ��������    
  }
}	  
//����IICֹͣ�ź�
void IIC_Stop(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	SDA_1_OUT();//sda1�����
	IIC_1_SCL=0;
	IIC_1_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_1_SCL=1; 
	IIC_1_SDA=1;//����I2C���߽����ź�
	delay_us(4);	
  }
  else
  {
	SDA_2_OUT();//sda2�����
	IIC_2_SCL=0;
	IIC_2_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_2_SCL=1; 
	IIC_2_SDA=1;//����I2C���߽����ź�
	delay_us(4);	
  }

}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(unsigned char IIC_Addr)
{
  u8 ucErrTime;
  
   if(IIC_Addr == LTC4015_IIC)
  {
	ucErrTime=0;
	SDA_1_IN();      //SDA1����Ϊ����  
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
	IIC_1_SCL=0;//ʱ�����0 	   
	return 0;  
  }
  else
  {
        ucErrTime=0;
	SDA_2_IN();      //SDA2����Ϊ����  
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
	IIC_2_SCL=0;//ʱ�����0 	   
	return 0;   
  }
} 
//����ACKӦ��
void IIC_Ack(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	IIC_1_SCL=0;
	SDA_1_OUT();
	IIC_1_SDA=0;
	delay_us(2);
	IIC_1_SCL=1;
	delay_us(2);
	IIC_1_SCL=0;
  }
  else
  {
        IIC_2_SCL=0;
	SDA_2_OUT();
	IIC_2_SDA=0;
	delay_us(2);
	IIC_2_SCL=1;
	delay_us(2);
	IIC_2_SCL=0;
  }
}
//������ACKӦ��		    
void IIC_NAck(unsigned char IIC_Addr)
{
  if(IIC_Addr == LTC4015_IIC)
  {
	IIC_1_SCL=0;
	SDA_1_OUT();
	IIC_1_SDA=1;
	delay_us(2);
	IIC_1_SCL=1;
	delay_us(2);
	IIC_1_SCL=0;
  }
  else
  {
        IIC_2_SCL=0;
	SDA_2_OUT();
	IIC_2_SDA=1;
	delay_us(2);
	IIC_2_SCL=1;
	delay_us(2);
	IIC_2_SCL=0;
  
  }
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(unsigned char IIC_Addr, u8 txd)
{                        
  u8 t;  
 if(IIC_Addr == LTC4015_IIC)
  {
    SDA_1_OUT(); 	    
    IIC_1_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_1_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)
                IIC_1_SDA=1;
        else
                IIC_1_SDA=0;
        txd<<=1; 	  
        delay_us(2);   //��TEA5767��������ʱ���Ǳ����
        IIC_1_SCL=1;
        delay_us(2); 
        IIC_1_SCL=0;	
        delay_us(2);
    }	
  }
 else
 {
    SDA_2_OUT(); 	    
    IIC_2_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_2_SDA=(txd&0x80)>>7;
        if((txd&0x80)>>7)
                IIC_2_SDA=1;
        else
                IIC_2_SDA=0;
        txd<<=1; 	  
        delay_us(2);   //��TEA5767��������ʱ���Ǳ����
        IIC_2_SCL=1;
        delay_us(2); 
        IIC_2_SCL=0;	
        delay_us(2);
    }	
 }
} 	  

//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char IIC_Addr, unsigned char ack)
{
  unsigned char i,receive=0;
    
  if(IIC_Addr == LTC4015_IIC)
  { 
    
    SDA_1_IN();//SDA1����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_1_SCL=0; 
        delay_us(2);
        IIC_1_SCL=1;
        receive<<=1;
        if(READ_1_SDA)receive++;   
        delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck(LTC4015_IIC);//����nACK
    else
        IIC_Ack(LTC4015_IIC); //����ACK   
    return receive;
  }
  else
  {
    SDA_2_IN();//SDA2����Ϊ����
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
        IIC_NAck(CAT5140_IIC);//����nACK
    else
        IIC_Ack(CAT5140_IIC); //����ACK   
    return receive;
  
  
  }
}



bool IIC_Write_Nbytes(unsigned char IIC_NUMB, u8 * data_ptr, u8 num_bytes)
{
  bool res =false;
  u8 t;
  for(t=0; t<num_bytes; t++)
  {
    IIC_Send_Byte(IIC_NUMB, *data_ptr);
    
    if(IIC_Wait_Ack(IIC_NUMB) == 0)
    {
      res = true;
    }
  
    data_ptr++;
  }
  
  return res;
}
bool IIC_Read_Nbytes(unsigned char IIC_NUMB, u8 * data_ptr, u8 num_bytes)
{
  bool res =false;
  u8 t;
  for(t=0; t<num_bytes; t++)
  {
    *data_ptr++ = IIC_Read_Byte(IIC_NUMB, 0);
    
    if(IIC_Wait_Ack(IIC_NUMB) == 0)
    {
      res = true;
    }
  
  }
  
  return res;

}


























