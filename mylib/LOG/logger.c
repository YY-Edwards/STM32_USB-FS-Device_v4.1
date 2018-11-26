#include "logger.h"


volatile RingQueue_t logger_msg_queue_ptr = NULL;
static volatile unsigned int logger_count = 0;

volatile unsigned char usart_send_buffer[256]= {0};
static volatile bool DMA_ALLOW_SEND_FLAG  = true;

DMA_InitTypeDef DMA_InitStructure;        //DMA��ʼ���ṹ��������ȫ��


void my_dma_config_and_enabled(void *p, uint16_t p_len)
{
  uint16_t send_len = p_len;
  
  if(send_len > sizeof(usart_send_buffer))
    send_len = sizeof(usart_send_buffer);
  
  //memcpy(usart_send_buffer, p, send_len);
  
  //while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);//�ȴ�DMA���ݴ������
  
  DMA_DeInit(DMA1_Channel4);//����DMA1��CH4
  //DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart_send_buffer;//����DMA��������ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)p;//����DMA��������ַ
  DMA_InitStructure.DMA_BufferSize = send_len;//Ҫ�������ݳ���
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);//��ʼ��DMA1��CH7
  
  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//ʹ�ܴ�������ж�
  
  DMA_Cmd(DMA1_Channel4, ENABLE);//��DMA��ͨ��4����ʱ�����ݾͷ��ͳ�ȥ�ˣ�

}


static void usart1_gpio_init()
{
  
    GPIO_InitTypeDef  GPIO_InitStructure;  
    
#if defined(STM32L1XX_MD)
    
      /* Peripheral clock enable */
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    
    //���ȿ������ã���ȷ�����ó���һ�֣���Ϊͬһ���˿ڿ���֧�ֶ��ָ��ù���
    //����1��Ӧ���Ÿ���ӳ��,��F1�Ķ˿���ӳ�䲻ͬ��
    
    //��F1�У�ͨ����ӳ��ķ�ʽ�����蹦�ܿ��Խ����蹦��ӳ�䵽��Ĭ�Ͻţ���Ҫ�����ֲᣬ�������ģ���
    //�����ֲַ���ӳ���ȫ��ӳ�䡣
    
    //����L1�У����蹦�ܿ���ͨ�����ü����ڲ�ͬ�Ĺܽ�ʵ�����蹦�ܡ�
    
    //�����֮��F1����ͨ�����ò�����ʱʵ���ڲ�ͬ�ܽŵ���ͬ���蹦�ܡ�
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* Push-pull or open drain */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* None, Pull-up or pull-down */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz; /* 400 KHz, 2, 10 or 40MHz */
    
    GPIO_Init(GPIOA, &GPIO_InitStructure);
     
    
#else
  
    
#endif  
  

}


static void usart1_dma_interrupt_config()
{
  
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;         //ͨ������ΪDMA1_4�ж�  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //�ж�ռ�ȵȼ�2 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //�ж���Ӧ���ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //���ж�  
  NVIC_Init(&NVIC_InitStructure);                                 //��ʼ��  

}

static void usart1_dma_init()
{
  
//  DMA_InitTypeDef DMA_InitStructure;        //DMA��ʼ���ṹ������
  
  USART_InitTypeDef USART_InitStructure;   //USART��ʼ���ṹ������
  
#if defined(STM32L1XX_MD)
  
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    
  /* USART1 configured as follow:  
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;               /*���ò�����Ϊ115200*/
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*��������λΪ8*/
  USART_InitStructure.USART_StopBits = USART_StopBits_1;     /*����ֹͣλΪ1λ*/
  USART_InitStructure.USART_Parity = USART_Parity_No;        /*����żУ��*/
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*��Ӳ������*/
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /*���ͺͽ���*/

  /*���ô���1 */
  USART_Init(USART1, &USART_InitStructure);
    
    
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//��DMA��������
  
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�򿪴��ڽ����ж�
  
  USART_Cmd(USART1, ENABLE);//�򿪴���
  
  
    /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		        //ʹ��DMAʱ��
  /* DMA1 channel4 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel4);		                                //����DMA1�ĵ�4ͨ��
  DMA_InitStructure.DMA_PeripheralBaseAddr              =  (uint32_t)(&USART1->DR);              //DMA��Ӧ���������ַ:USART1
  DMA_InitStructure.DMA_MemoryBaseAddr                  = (uint32_t)usart_send_buffer;                              //�ڴ�洢����ַ
  DMA_InitStructure.DMA_DIR                             = DMA_DIR_PeripheralDST;	                //DMA��ת��ģʽΪDSTģʽ�����ڴ���Ƶ�����
  DMA_InitStructure.DMA_BufferSize                      = sizeof(usart_send_buffer);		                 //DMA�����С��1��
  DMA_InitStructure.DMA_PeripheralInc                   = DMA_PeripheralInc_Disable;	//����һ�����ݺ��豸��ַ��ֹ����
  DMA_InitStructure.DMA_MemoryInc                       = DMA_MemoryInc_Enable;	        //��������һ�����ݺ�Ŀ���ڴ��ַ����
  DMA_InitStructure.DMA_PeripheralDataSize              = DMA_PeripheralDataSize_Byte;  //�����������ݿ��Ϊ8λ
  DMA_InitStructure.DMA_MemoryDataSize                  = DMA_MemoryDataSize_Byte;       //DMA�������ݳߴ磬Byte����Ϊ8λ
  DMA_InitStructure.DMA_Mode                            = DMA_Mode_Normal;                         //ת��ģʽ��ѭ������ģʽ��
  DMA_InitStructure.DMA_Priority                        = DMA_Priority_High;	                //DMA���ȼ���
  DMA_InitStructure.DMA_M2M                             = DMA_M2M_Disable;		                //M2Mģʽ����
  
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);  
  
  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//ʹ�ܴ�������ж�
  
  /* Enable DMA1 channel4 */
  //DMA_Cmd(DMA1_Channel4, ENABLE);
  
#else
    
#endif
    
    
}

static void logger_output_peripheral_init()
{
  
  usart1_gpio_init();
  
  usart1_dma_init();
  
  usart1_dma_interrupt_config();//DMA�ж�����
  

}

void usart_output_log_task(void *p)//���������ݽ�ȥ
{
    unsigned short msg_len = 0;
    bool ret = false;
  
    if(DMA_ALLOW_SEND_FLAG == true)
    {   
        ret = logger_output_msg(p, &msg_len);
        if(ret == true)//��ȡ���ݳɹ�
        {
          if(msg_len >256)msg_len =256;
          
          DMA_ALLOW_SEND_FLAG = false;
          
          my_dma_config_and_enabled(p, msg_len);//����DMA����
                    
        }
        else//logger�����������ݣ������κδ���
        {
          
        }  
    }
}

void DMA1_Channel4_IRQHandler(void)
{

   if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
  {
    //�����־λ  	
    DMA_ClearFlag(DMA1_FLAG_TC4);	
    
    //DMA_ClearITPendingBit(DMA1_FLAG_TC4);  	
    //DMA1->IFCR |= DMA1_FLAG_TC4;	
    //�ر�DMA	
    DMA_Cmd(DMA1_Channel4, DISABLE); 
    
    //DMA1_Channel4->CCR &= ~(1<<0); 
    
    //�����ٴη���	
    DMA_ALLOW_SEND_FLAG = true;
    //Flag_Uart_Send = 0;
  }

}

extern void set_timer_task(unsigned char       timer_id, 
                            unsigned int        delay, 
                            unsigned char       rearm, 
                            handler              timehandler, 
                            void *               param );

void logger_init()
{
  
  logger_msg_queue_ptr = create_queue(17, 128);
  if(logger_msg_queue_ptr == NULL)
  {
    return;
  }
  

  logger_output_peripheral_init();
  
  set_timer_task(LOGGER_TASK, TIME_BASE_10MS*2, true, usart_output_log_task, (void *)usart_send_buffer);
  
}

void logger_add_msg_to_queue(const char* psz_level,
                                const char* psz_file,
                                int line_no,
                                const char* psz_funcsig,
                                const char *psz_fmt, ...)
{
        
        
  	char msg[256] = { 0 };
	//C�����н����������һ���,����ͷ�ļ���#include <stdarg.h>,���ڻ�ȡ��ȷ�������Ĳ��� 
	va_list vArgList;

	//��va_list�������г�ʼ������vArgListָ��ָ������б��еĵ�һ������
	va_start(vArgList, psz_fmt);

	//��vArgList(ͨ�����ַ���) ��format��ʽд���ַ���string��
	vsnprintf(msg, 256, psz_fmt, vArgList);
	//����vArgListָ��
	va_end(vArgList);

        logger_count++;//��־����
        if(logger_count == 0xffffffff)
        {
          logger_count = 0;
        }
	char content[280] = { 0 };
	sprintf(content,
		//"[%d][%s][%s:%d %s]: %s\r\n",
                "[%d][%s][%s:%d]: %s\r\n",
		logger_count,
		psz_level,
                //psz_file,
		//line_no,
		psz_funcsig,
                line_no,
		msg);



	{
            if(logger_msg_queue_ptr !=NULL)
            {
              bool ret =false;
              ret = push_to_queue(logger_msg_queue_ptr, content, (strlen(content)+1));
              if(ret !=true)
              {
                memset(content, 0x00, sizeof(content));
                //log_warning("push log err!");�ݹ���ѭ���ˣ�
              }
            }
	
	}

}

bool logger_output_msg(void *buf, unsigned short *buf_len)
{
  bool ret =false;
  if(logger_msg_queue_ptr !=NULL)
  {
     ret = take_from_queue(logger_msg_queue_ptr, buf, (int *)buf_len, true);
  }
  return ret;
}