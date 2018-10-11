/**
  ******************************************************************************
  * @file    stm32_it.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
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
#include "hw_config.h"
#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include <string.h>
#include "stdbool.h"
static int check_count = 0;  
static unsigned char PTT_Sta = 1;//Ĭ�ϸߵ�ƽ
extern __IO uint32_t packet_sent;  
extern __IO uint32_t bDeviceState; 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
//int timer3_counter_value =0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void SysTick_Handler(void)
{
}

//void SysTick_Handler(void)
//{
//  check_count++;
//  if(check_count >= 8)
//  {
//    check_count =0;
//    static u8 Keybuf1 = 0xff;
////    
////    //PA8,������PA8
////    Keybuf1 = ( ( Keybuf1 << 1 ) | GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) );//����������1λ��������ǰֵ�������λ
////
////    if ( 0x00 == Keybuf1 )//����8��ɨ�趼Ϊ0����16�����ڶ���⵽����״̬������Ϊ��������
////    {
////      PTT_Sta = 0;
////    }
////    else if ( 0xff == Keybuf1 )//��������
////    {
////      PTT_Sta = 1;
////    }
////    else//���������˵������״̬��δ�ȶ����򲻶� KeySta ����ֵ���и���
////    //Key1Sta = 1;//default value 
////    {
////    }
//    
//  } 
//}

/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)|| defined (STM32F37X)
void USB_LP_IRQHandler(void)
#else
void USB_LP_CAN1_RX0_IRQHandler(void)
#endif
{
  USB_Istr();//���� USB �����ĸ����жϣ�
}

/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
void USB_FS_WKUP_IRQHandler(void)
#else
void USBWakeUp_IRQHandler(void)
#endif
{
  EXTI_ClearITPendingBit(EXTI_Line18);//����жϱ�־,�͹������ڻ����豸��
}


void TIM2_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
     check_count++;
     if(check_count >= 8)
    {
      check_count =0;
      static u8 Keybuf1 = 0xff;
      
      //PA3
      Keybuf1 = ( ( Keybuf1 << 1 ) | GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) );//����������1λ��������ǰֵ�������λ

      if ( 0x00 == Keybuf1 )//����8��ɨ�趼Ϊ0����16�����ڶ���⵽����״̬������Ϊ��������
      {
        PTT_Sta = 0;
      }
      else if ( 0xff == Keybuf1 )//��������
      {
        PTT_Sta = 1;
      }
      else//���������˵������״̬��δ�ȶ����򲻶� KeySta ����ֵ���и���
      //Key1Sta = 1;//default value 
      {
      }
      
    } 

    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }

}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
  bool ret = false;
  static unsigned char msg[256] ={0};
  static int remain_len =0;
  static int send_count =0 ;
  int msg_len = 0;
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    //timer3_counter_value++;
    if(bDeviceState == CONFIGURED)//�����ɹ�ʶ�����������
    {
      if (packet_sent == 1)//������
      {
        if(remain_len !=0)//����ʣ������
        {
          if(remain_len >64)
          {
             CDC_Send_DATA (&msg[64*send_count], 64);
             remain_len -= 64;
             send_count++;
          }
          else
          {
            CDC_Send_DATA (&msg[64*send_count], remain_len);
            remain_len = 0;
            send_count = 0;
            //memset(msg, 0x00, sizeof(msg));�˴����ܻᵼ�·���δ��ɣ�����ǰ��ջ�������ݡ�
          }
      
        }
        else//�Ӷ����ж����µ���־����,�ȴ���һ�η���
        {
          memset(msg, 0x00, sizeof(msg));
          ret = logger_output_msg(msg, &msg_len);
          if(ret == true)//��ȡ���ݳɹ�
          {
            if(msg_len >256)msg_len =256;
            remain_len = msg_len;
          }
        }  
      }
   
//      if(packet_sent == 1)
//      {
////          unsigned char temp_send = PTT_Sta;
////          FootPtt_Pro_t send_pro;
////          memset((void*)&send_pro, 0x00, sizeof(FootPtt_Pro_t));
////          send_pro.header       = PRO_HEADER;
////          send_pro.opcode       = PRO_OPCODE_PTT;
////          send_pro.length       = 0x01;
////          send_pro.payload      = temp_send;
////          send_pro.checksum     = (send_pro.opcode + send_pro.length + send_pro.payload);
////          send_pro.end          = PRO_END;           
////          CDC_Send_DATA ((unsigned char*)&send_pro,sizeof(FootPtt_Pro_t));
//      }
    
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }

}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

