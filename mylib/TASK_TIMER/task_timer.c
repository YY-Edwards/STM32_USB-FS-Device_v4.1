
#include "task_timer.h"


#define MAX_TASKS       7
#define TASK_TIEMR_BASE_MS   10   //10MS


#pragma pack(1)

  
typedef struct
{       
      bool              is_run;//indicate that whether the task is running.
      unsigned short    slice_count;//Execution cycle 
      unsigned short    reset_count;//reset slice_count in reset_count value when slice_count is zero.
      handler           timer_handler;//task function pointer.
      void *            param;//function parameter pointer.
} task_info_t;

#pragma pack()

 volatile task_info_t tasks[MAX_TASKS];//定义任务结构体
 
 void set_timer_task(unsigned char timer_id, 
                     unsigned int delay, 
                     unsigned char rearm, 
                     handler timehandler, 
                     void *param )
 {
      if(timer_id < (MAX_TASKS-1))
      {
        tasks[timer_id].is_run           = false;
        tasks[timer_id].reset_count      = rearm?delay:0;
        tasks[timer_id].slice_count      = delay;
        tasks[timer_id].timer_handler    = timehandler;
        tasks[timer_id].param            = param;
      }
      //当delay =0 ；rearm=false时，则暂停指定任务
 }
 
 
 
 
 static void task_reset()
 {
   
   for(unsigned char i = 0; i <MAX_TASKS; i++)
   {
      memset((void*)&tasks[i], 0x00, sizeof(task_info_t));
   }
 
 }
 
 
 
//通用定时器 3 中断初始化

//这里时钟选择为 APB1 的 2 倍，而 APB1 为 36M(当SYSCLK==72M时，SYSCLK2分频得到36M的APB1)
//#if defined(STM32L1XX_MD)
//这里时钟选择为 APB1，而 APB1 为 32M(当SYSCLK==32M时，1分频得到32M的APB1)
//#endif

//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器 3!
static void TIM3_Int_Init(uint16_t arr,uint16_t psc)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //①时钟 TIM3 使能
  //定时器 TIM3 初始化
  TIM_TimeBaseStructure.TIM_Period = arr; //设置自动重装载寄存器周期的值
  TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置时钟频率除数的预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割
  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //②初始化 TIM3
  
  //这样后可以修正：
  //在用到STM32定时器的更新中断时，发现有些情形下只要开启定时器就立即进入一次中断。
  //准确说，只要使能更新中断允许位就立即响应一次更新中断【当然前提是相关NVIC也已经配置好】
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
  
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //③允许更新中断
  //中断优先级 NVIC 设置
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//0; //先占优先级 1 级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //从优先级 1级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道被使能
  NVIC_Init(&NVIC_InitStructure); //④初始化 NVIC 寄存器
  
//  TIM_Cmd(TIM3, ENABLE); //⑤使能 TIM3
  
}

volatile void timer_task_schedule_func(void)
{
    
  for (unsigned char i=0; i < MAX_TASKS; ++i) //遍历任务数组        
  {            
    if (tasks[i].slice_count !=0)  //定时任务使能            
    { 
      tasks[i].slice_count -= TASK_TIEMR_BASE_MS;
      if (tasks[i].slice_count == 0) //时间片到了                 
      { 
        if(tasks[i].timer_handler)
        {
          tasks[i].is_run = true;//置位  表示任务可以执行 
          tasks[i].slice_count = tasks[i].reset_count; //重新加载时间片值，为下次做准备                 
        } 
      }
    }        
  }

}
void start_timer_task_schedule()
{ 
  TIM_Cmd(TIM3, ENABLE); //⑤使能 TIM3
}

void stop_timer_task_schedule()
{
  TIM_Cmd(TIM3, DISABLE); //⑤失能 TIM3
}

void task_process()
{ 
    unsigned short i = 0;   
    for (i=0; i < MAX_TASKS; ++i) //遍历任务数组     
    {         
      if (tasks[i].is_run == true) //若任务可执行，则执行任务         
      {   
        void *p = tasks[i].param;
        tasks[i].timer_handler(p);             
        tasks[i].is_run = false;//将标志位清零        
      }    
    }

}


void task_init()
{

  task_reset();
#if defined(STM32L1XX_MD) 
  
   TIM3_Int_Init((TASK_TIEMR_BASE_MS*10-1),3199);//定时输出log,或者做任务调度的基准时钟  
    //Tout= ((arr+1)*( psc+1))/Tclk=(99+1)*( 3199+1))/32=10ms
#else    
   TIM3_Int_Init(99,7199); //10Khz 的计数频率，计数到 500 为 50ms
   //Tout= ((799+1)*( 7199+1))/72=500000us=80ms
#endif 
   
}
 
 