/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Virtual Com Port Loopback Demo main file
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
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "delay.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEM_CLOCK 72000000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  delay_init(72);//延时功能初始化
  //SysTick_Config(SYSTEM_CLOCK / 500);//2ms
  STM_EVAL_LEDInit(LED2);//根据原理图，修改函数里的LED宏定义即可
  STM_EVAL_LEDInit(LED3);
  
  STM_EVAL_LEDOn(LED2);//高电平，点亮
  STM_EVAL_LEDOn(LED3);
  
  Set_System();
  Set_USBClock();// 配置 USB 时钟，也就是从 72M 的主频得到 48M 的 USB 时钟（1.5 分频）
  USB_Interrupts_Config();// USB 唤醒中断和USB 低优先级数据处理中断
  USB_Init();//用于初始化 USB，;
  
  TIM2_Int_Init(20, 7199);//2ms
  TIM3_Int_Init(999,7199); //10Khz 的计数频率，计数到 1000 为 100ms
                        //Tout= ((799+1)*( 7199+1))/72=500000us=80ms
  unsigned int run_counts = 0;
  unsigned char timer3_run_flag = 0;
  while (1)
  {
    if (bDeviceState == CONFIGURED)
    {
      run_counts++;
      if(timer3_run_flag == 0)
      {
        TIM_Cmd(TIM3, ENABLE);//启动定时器3
        timer3_run_flag = 1;
      }
      CDC_Receive_DATA();
      /*Check to see if we have data yet */
      if (Receive_length  != 0)
      {
//        if (packet_sent == 1)
//          CDC_Send_DATA ((unsigned char*)Receive_Buffer,Receive_length);
       Receive_length = 0;
      }
      if(run_counts == 60000)
      {
        STM_EVAL_LEDToggle(LED2);
        run_counts = 0;
      }
    }
    else
    {
      STM_EVAL_LEDToggle(LED3);   
      delay_ms(100); 
    }
  }
} 

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
