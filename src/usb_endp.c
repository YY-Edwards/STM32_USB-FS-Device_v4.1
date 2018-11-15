/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.1.0
  * @date    26-May-2017
  * @brief   Endpoint routines
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
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

#include "stdbool.h"
#include "myqueue.h"    
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t packet_sent;
extern __IO uint32_t packet_receive;
extern volatile RingQueue_t usb_rx_queue_ptr;
uint32_t Receive_length;

//uint16_t In_Data_Offset;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void EP1_IN_Callback (void)
{
  packet_sent = 1;//���ͳɹ��ı�־
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
  
  uint16_t Data_Len;       /* data length*/
  uint8_t Receive_Buffer[64];
  
  if (GetENDPOINT(ENDP3) & EP_DTOG_TX)
  {
    //���ͷ��û��Ի�������ռ�У������Ļ�USB����һ�����չ��̿������̽��У�
    //����һ�黺������ͬʱ�û����н������洦������һ�������֮ǰ��
    //�������û����ݾ��У����򻺳���������
    FreeUserBuffer(ENDP3, EP_DBUF_OUT);//�������ڴ�������ǰ����ֽ��տհ�����𣿴���֤
    /*read from ENDP3_BUF0Addr buffer*/
    Data_Len = GetEPDblBuf0Count(ENDP3);
    PMAToUserBufferCopy((unsigned char*)Receive_Buffer, ENDP3_BUF0Addr, Data_Len);
  }
  else
  {
    FreeUserBuffer(ENDP3, EP_DBUF_OUT);
    /*read from ENDP3_BUF1Addr buffer*/
    Data_Len = GetEPDblBuf1Count(ENDP3);
    PMAToUserBufferCopy((unsigned char*)Receive_Buffer, ENDP3_BUF1Addr, Data_Len);
  }
  
  
  if(usb_rx_queue_ptr != NULL)
  {
    bool ret =false;
    ret = push_to_queue(usb_rx_queue_ptr, Receive_Buffer, Data_Len);
    if(ret !=true)
    {
    }
  }
  SetEPRxValid(ENDP3); //�������ö˵����״̬��Ч����Ϊ���������ݺ󣬶˵�ͻᱻ�ر�.
  
  
//  packet_receive = 1;
//  Receive_length = GetEPRxCount(ENDP3);//��ȡ����PC�˷��͵����ݳ���
//  //�����յ������ݴ�PMA����ȡ���ŵ��Զ��建����
//  PMAToUserBufferCopy((unsigned char*)Receive_Buffer, ENDP3_RXADDR, Receive_length);


}



void RESET_Callback(void)
{
    packet_sent = 1;//���ͳɹ��ı�־
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
