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

#include "logger.h"
#include "BNP.h" 
#include "task_timer.h"    

extern __IO uint32_t bDeviceState; 

//void set_system()
//{
//  #if defined(STM32L1XX_MD)
//  
//   /* PLL_VCO = HSE_VALUE * PLL_MUL = 96 MHz */
//  /* USBCLK = PLL_VCO / 2= 48 MHz */
//  /* SYSCLK = PLL_VCO * PLL_DIV = 32 MHz */
//  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLDIV3);
//  /* Enable PLL */
//  RCC->CR |= RCC_CR_PLLON;
//  /* Wait till PLL is ready */
//  while((RCC->CR & RCC_CR_PLLRDY) == 0)
//  { }
//  /* Select PLL as system clock source */
//  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
//  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
//  /* Wait till PLL is used as system clock source */
//  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
//  { }
//  
//  
//  delay_init(32);
//  //同时启用有冲突。
//  //SYSCLK = 32M
//  //SysTick_Config(32000000 / 500);//2ms
//  
//#else
//   delay_init(72);//延时功能初始化
//   //SysTick_Config(SYSTEM_CLOCK / 500);//2ms
//#endif  
//
//   
//}

/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  //set_system();//配置系统时钟等
  
  hardware_init();//外设初始化
  
  protocol_init();//协议初始化

  unsigned int run_counts = 0;
  while (1)
  {
    if (bDeviceState == CONFIGURED)
    //if (1)
    {
      
      start_timer_task_schedule();
      
      log_debug("start task schedule.");
      
      while(bDeviceState == CONFIGURED)
      {
        
        task_process();
        run_counts++;
        if(run_counts == 10*60000)
        {
          STM_EVAL_LEDToggle(LED2);
          run_counts = 0;
        }
      }
      
      log_warning("stop task schedule!");
      stop_timer_task_schedule();
      
    }
    else
    {
      STM_EVAL_LEDToggle(LED3);//指示等待上位机识别   
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
