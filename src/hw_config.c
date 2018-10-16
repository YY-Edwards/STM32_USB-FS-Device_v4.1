/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;
extern __IO uint32_t packet_sent;
extern __IO uint8_t Send_Buffer[VIRTUAL_COM_PORT_DATA_SIZE] ;
extern __IO  uint32_t packet_receive;
extern __IO uint8_t Receive_length;

uint8_t Receive_Buffer[64];
uint32_t Send_length;
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;  
  
#if defined(STM32L1XX_MD)
  
    /* Enable integrated STM32L15xx internal pull up 
  Enable the SYSCFG module clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  
   /* Configure USB DM/DP pin. This is optional, and maintained only for user guidance.
  For the STM32L products there is no need to configure the PA12/PA11 pins couple 
  as Alternate Function */
  
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);

//����Ҫ���ã�
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USB);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USB);
  
  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* Push-pull or open drain */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* None, Pull-up or pull-down */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz; /* 400 KHz, 2, 10 or 40MHz */
  GPIO_Init(GPIOA, &GPIO_InitStructure);
//  
  STM32L15_USB_DISCONNECT;
  //GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  
#else
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /********************************************/
  /*  Configure USB DM/DP pins                */
  /********************************************/

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* Enable all GPIOs Clock*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ALLGPIO, ENABLE);

  
    /* Enable the USB disconnect GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
  
  /* USB_DISCONNECT used as USB pull-up */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//GPIO_Mode_Out_OD,
  //�ӵ���������������Ŀ�©����ǿ��Եģ���Ϊ�ⲿͨ�������ܲ������������
  //һ����˵����©���������Ӳ�ͬ��ƽ��������ƥ���ƽ�õģ���Ϊ��©���Ų������ⲿ����������ʱ��ֻ������͵�ƽ�������Ҫͬʱ�߱�����ߵ�ƽ�Ĺ��ܣ�����Ҫ���������裬�ܺõ�һ���ŵ���ͨ���ı�������Դ�ĵ�ѹ������Ըı䴫���ƽ��
  //���������������Ϳ����ṩTTL/CMOS��ƽ�����.
  //���Լ��Ŀ�������������������ͨ���������PG11�ĸߵ͵�ƽ���������PCʶ��
 
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
  
  GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);//�����ͣ��ȳ�ʼ����ɺ������ߣ�
                                                    //�Ա�����ʶ����ٷ�������
                                                    //����MCU�ſ�������Ӧ��
#endif
 
  
  
#ifdef USB_LOW_PWR_MGMT_SUPPORT
  
  /**********************************************************************/
  /*  Configure the EXTI line 18 connected internally to the USB IP     */
  /**********************************************************************/
  
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

#endif  /* USB_LOW_PWR_MGMT_SUPPORT */
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS) 
   
  /* Enable USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
  
#else 
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  
  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32L1XX_MD */
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }
    /*Enable SystemCoreClock*/
  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
 
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
    /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#else
  
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure); 
#endif
}



//ͨ�ö�ʱ�� 3 �жϳ�ʼ��
//����ʱ��ѡ��Ϊ APB1 �� 2 ������ APB1 Ϊ 36M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ�� 3!
void TIM3_Int_Init(uint16_t arr,uint16_t psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //��ʱ�� TIM3 ʹ��
  //��ʱ�� TIM3 ��ʼ��
  TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler =psc; //����ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�
  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ���ϼ���
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //�ڳ�ʼ�� TIM3
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //����������ж�
  //�ж����ȼ� NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 �ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ� 0 ��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //�����ȼ� 1��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure); //�ܳ�ʼ�� NVIC �Ĵ���
  
  //TIM_Cmd(TIM3, DISABLE); //��ʹ�� TIM3
  
}


void TIM2_Int_Init(uint16_t arr,uint16_t psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //��ʱ�� TIM2 ʹ��
  //��ʱ�� TIM2 ��ʼ��
  TIM_TimeBaseStructure.TIM_Period = arr; //�����Զ���װ�ؼĴ������ڵ�ֵ
  TIM_TimeBaseStructure.TIM_Prescaler =psc; //����ʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�
  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM ���ϼ���
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //�ڳ�ʼ�� TIM2
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //����������ж�
  //�ж����ȼ� NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; //TIM2 �ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ� 0 ��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ� 0��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure); //�ܳ�ʼ�� NVIC �Ĵ���
  
  TIM_Cmd(TIM2, ENABLE); //��ʹ�� TIM2
  
}

#if !defined (USE_NUCLEO)
/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#if defined(STM32L1XX_MD) || defined (STM32L1XX_HD)|| (STM32L1XX_MD_PLUS)
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }  
  
#else 
  if (NewState != DISABLE)
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);//�������3�ſ����壬�˴��豸��ʼ����ɣ�����
                                                      //��������������ʶ��
    //GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);//�˴��ø߻����õ͸��ݵ�������������
  }
  else
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
    //GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif
}
#endif /* USE_NUCLEO */

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
  uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

  Device_Serial0 = *(uint32_t*)ID1;
  Device_Serial1 = *(uint32_t*)ID2;
  Device_Serial2 = *(uint32_t*)ID3;
 
  Device_Serial0 += Device_Serial2;

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;
  
  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10; 
    }
    
    value = value << 4;
    
    pbuf[ 2* idx + 1] = 0;
  }
}

/*******************************************************************************
* Function Name  : Send DATA .
* Description    : send the data received from the STM32 to the PC through USB  
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Send_DATA (uint8_t *ptrBuffer, uint8_t Send_length)
{
  /*if max buffer is Not reached*/
  if(Send_length <= VIRTUAL_COM_PORT_DATA_SIZE)     
  {
    /*Sent flag*/
    packet_sent = 0;
    /* send  packet to PMA*/
    UserToPMABufferCopy((unsigned char*)ptrBuffer, ENDP1_TXADDR, Send_length);
    SetEPTxCount(ENDP1, Send_length);//�Ӷ˵�1����Send_length�ֽ�����
    SetEPTxValid(ENDP1);//ʹ�ܶ˵�1�ķ���״̬
  }
  else
  {
    return 0;
  } 
  return 1;
}

/*******************************************************************************
* Function Name  : Receive DATA .
* Description    : receive the data from the PC to STM32 and send it through USB
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Receive_DATA(void)
{ 
  /*Receive flag*/
  packet_receive = 0;
  SetEPRxValid(ENDP3); //�������ö˵����״̬��Ч����Ϊ���������ݺ󣬶˵�ͻᱻ�ر�.
  return 1 ;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
