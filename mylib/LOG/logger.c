#include "logger.h"


volatile RingQueue_t logger_msg_queue_ptr = NULL;
static volatile unsigned int logger_count = 0;

volatile unsigned char usart_send_buffer[256]= {0};
static volatile bool DMA_ALLOW_SEND_FLAG  = true;

DMA_InitTypeDef DMA_InitStructure;        //DMA初始化结构体声明：全局


void my_dma_config_and_enabled(void *p, uint16_t p_len)
{
  uint16_t send_len = p_len;
  
  if(send_len > sizeof(usart_send_buffer))
    send_len = sizeof(usart_send_buffer);
  
  //memcpy(usart_send_buffer, p, send_len);
  
  //while (DMA_GetFlagStatus(DMA1_FLAG_TC4) == RESET);//等待DMA数据传输完毕
  
  DMA_DeInit(DMA1_Channel4);//重置DMA1的CH4
  //DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)usart_send_buffer;//设置DMA缓冲区地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)p;//设置DMA缓冲区地址
  DMA_InitStructure.DMA_BufferSize = send_len;//要传送数据长度
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);//初始化DMA1的CH7
  
  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//使能传输完成中断
  
  DMA_Cmd(DMA1_Channel4, ENABLE);//打开DMA的通道4，这时候数据就发送出去了！

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
    
    //首先开启复用，再确定复用成哪一种，因为同一个端口可能支持多种复用功能
    //串口1对应引脚复用映射,与F1的端口重映射不同。
    
    //在F1中，通过重映射的方式，外设功能可以将外设功能映射到非默认脚（需要根据手册，不是随便的），
    //其中又分部分映射和全部映射。
    
    //而在L1中，外设功能可以通过复用即可在不同的管脚实现外设功能。
    
    //简而言之，F1不能通过复用操作来时实现在不同管脚的相同外设功能。
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
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;         //通道设置为DMA1_4中断  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;       //中断占先等级2 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //中断响应优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //打开中断  
  NVIC_Init(&NVIC_InitStructure);                                 //初始化  

}

static void usart1_dma_init()
{
  
//  DMA_InitTypeDef DMA_InitStructure;        //DMA初始化结构体声明
  
  USART_InitTypeDef USART_InitStructure;   //USART初始化结构体声明
  
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
  USART_InitStructure.USART_BaudRate = 115200;               /*设置波特率为115200*/
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;/*设置数据位为8*/
  USART_InitStructure.USART_StopBits = USART_StopBits_1;     /*设置停止位为1位*/
  USART_InitStructure.USART_Parity = USART_Parity_No;        /*无奇偶校验*/
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*无硬件流控*/
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /*发送和接收*/

  /*配置串口1 */
  USART_Init(USART1, &USART_InitStructure);
    
    
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);//打开DMA发送请求
  
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//打开串口接收中断
  
  USART_Cmd(USART1, ENABLE);//打开串口
  
  
    /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		        //使能DMA时钟
  /* DMA1 channel4 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel4);		                                //开启DMA1的第4通道
  DMA_InitStructure.DMA_PeripheralBaseAddr              =  (uint32_t)(&USART1->DR);              //DMA对应的外设基地址:USART1
  DMA_InitStructure.DMA_MemoryBaseAddr                  = (uint32_t)usart_send_buffer;                              //内存存储基地址
  DMA_InitStructure.DMA_DIR                             = DMA_DIR_PeripheralDST;	                //DMA的转换模式为DST模式，由内存搬移到外设
  DMA_InitStructure.DMA_BufferSize                      = sizeof(usart_send_buffer);		                 //DMA缓存大小，1个
  DMA_InitStructure.DMA_PeripheralInc                   = DMA_PeripheralInc_Disable;	//接收一次数据后，设备地址禁止后移
  DMA_InitStructure.DMA_MemoryInc                       = DMA_MemoryInc_Enable;	        //开启接收一次数据后，目标内存地址后移
  DMA_InitStructure.DMA_PeripheralDataSize              = DMA_PeripheralDataSize_Byte;  //定义外设数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize                  = DMA_MemoryDataSize_Byte;       //DMA搬移数据尺寸，Byte就是为8位
  DMA_InitStructure.DMA_Mode                            = DMA_Mode_Normal;                         //转换模式，循环缓存模式。
  DMA_InitStructure.DMA_Priority                        = DMA_Priority_High;	                //DMA优先级高
  DMA_InitStructure.DMA_M2M                             = DMA_M2M_Disable;		                //M2M模式禁用
  
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);  
  
  DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);//使能传输完成中断
  
  /* Enable DMA1 channel4 */
  //DMA_Cmd(DMA1_Channel4, ENABLE);
  
#else
    
#endif
    
    
}

static void logger_output_peripheral_init()
{
  
  usart1_gpio_init();
  
  usart1_dma_init();
  
  usart1_dma_interrupt_config();//DMA中断配置
  

}

void usart_output_log_task(void *p)//将参数传递进去
{
    unsigned short msg_len = 0;
    bool ret = false;
  
    if(DMA_ALLOW_SEND_FLAG == true)
    {   
        ret = logger_output_msg(p, &msg_len);
        if(ret == true)//读取数据成功
        {
          if(msg_len >256)msg_len =256;
          
          DMA_ALLOW_SEND_FLAG = false;
          
          my_dma_config_and_enabled(p, msg_len);//启动DMA传输
                    
        }
        else//logger缓冲区无数据，则不作任何处理。
        {
          
        }  
    }
}

void DMA1_Channel4_IRQHandler(void)
{

   if (DMA_GetITStatus(DMA1_IT_TC4) != RESET)
  {
    //清除标志位  	
    DMA_ClearFlag(DMA1_FLAG_TC4);	
    
    //DMA_ClearITPendingBit(DMA1_FLAG_TC4);  	
    //DMA1->IFCR |= DMA1_FLAG_TC4;	
    //关闭DMA	
    DMA_Cmd(DMA1_Channel4, DISABLE); 
    
    //DMA1_Channel4->CCR &= ~(1<<0); 
    
    //允许再次发送	
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
	//C语言中解决变参问题的一组宏,所在头文件：#include <stdarg.h>,用于获取不确定个数的参数 
	va_list vArgList;

	//对va_list变量进行初始化，将vArgList指针指向参数列表中的第一个参数
	va_start(vArgList, psz_fmt);

	//将vArgList(通常是字符串) 按format格式写入字符串string中
	vsnprintf(msg, 256, psz_fmt, vArgList);
	//回收vArgList指针
	va_end(vArgList);

        logger_count++;//日志计数
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
                //log_warning("push log err!");递归死循环了？
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