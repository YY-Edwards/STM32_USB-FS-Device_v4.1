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
#include "logger.h"
#include "task_timer.h"
//static int check_count = 0;  
//static unsigned char PTT_Sta = 1;//默认高电平
//extern __IO uint32_t packet_sent;  
extern __IO uint32_t bDeviceState; 
extern volatile bool timeout_for_once;
extern void stop_bnp_timeout_detect(void);

//extern volatile unsigned char usart_send_buffer[256];

//static volatile bool DMA_ALLOW_SEND_FLAG  = true;
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
//  timer_task_schedule();//调度任务
}


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
  USB_Istr();//处理 USB 发生的各种中断，
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
  EXTI_ClearITPendingBit(EXTI_Line18);//清除中断标志,低功耗用于唤醒设备的
}


void TIM2_IRQHandler(void)
{

  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//3s
  {
    if(timeout_for_once == false)
      timeout_for_once = true;
    
    stop_bnp_timeout_detect();//关闭bnp response监测
    
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
 
    //unsigned short msg_len = 0;
   // bool ret = false;
    
#if defined (USE_TIMER_TASK)  
    
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    timer_task_schedule_func();//调度任务
    
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
       
#elif defined (USE_USART_LOGGER) 
  
  
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    if(DMA_ALLOW_SEND_FLAG == true)
    {
      
        ret = logger_output_msg((void*)usart_send_buffer, &msg_len);
        if(ret == true)//读取数据成功
        {
          if(msg_len >256)msg_len =256;
          
          DMA_ALLOW_SEND_FLAG = false;
          
          my_dma_config_and_enabled(NULL, msg_len);//启动DMA传输
                    
        }
        else//logger缓冲区无数据，则不作任何处理。
        {
          
        }  
    }
      
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
  
  
  
#elif defined (USE_CDC_LOGGER)

  static unsigned char msg[256] ={0};
  static int remain_len =0;
  static int send_count =0 ;
  if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
    //timer3_counter_value++;
    if(bDeviceState == CONFIGURED)//主机成功识别并以配置完成
    {
      if (packet_sent == 1)//允许发送
      {
        if(remain_len !=0)//发送剩余数据
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
            //memset(msg, 0x00, sizeof(msg));此处可能会导致发送未完成，而提前清空缓冲包数据。
          }
      
        }
        else//从队列中读出新的日志数据,等待下一次发送
        {
          memset(msg, 0x00, sizeof(msg));
          ret = logger_output_msg(msg, &msg_len);
          if(ret == true)//读取数据成功
          {
            if(msg_len >256)msg_len =256;
            remain_len = msg_len;
          }
        }  
      }
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
  }
  
#endif

}



//void DMA1_Channel4_IRQHandler(void)
//{
//
//   if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
//  {
//    //清除标志位  	
//    DMA_ClearFlag(DMA1_FLAG_TC4);	
//    
//    //DMA_ClearITPendingBit(DMA1_FLAG_TC4);  	
//    //DMA1->IFCR |= DMA1_FLAG_TC4;	
//    //关闭DMA	
//    DMA_Cmd(DMA1_Channel4, DISABLE); 
//    
//    //DMA1_Channel4->CCR &= ~(1<<0); 
//    
//    //允许再次发送	
//    DMA_ALLOW_SEND_FLAG = true;
//    //Flag_Uart_Send = 0;
//  }
//
//}



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

