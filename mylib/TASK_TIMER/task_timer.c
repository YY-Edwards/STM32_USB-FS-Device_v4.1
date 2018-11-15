
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

 volatile task_info_t tasks[MAX_TASKS];//��������ṹ��
 
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
      //��delay =0 ��rearm=falseʱ������ָͣ������
 }
 
 
 
 
 static void task_reset()
 {
   
   for(unsigned char i = 0; i <MAX_TASKS; i++)
   {
      memset((void*)&tasks[i], 0x00, sizeof(task_info_t));
   }
 
 }
 
 
 
//ͨ�ö�ʱ�� 3 �жϳ�ʼ��

//����ʱ��ѡ��Ϊ APB1 �� 2 ������ APB1 Ϊ 36M(��SYSCLK==72Mʱ��SYSCLK2��Ƶ�õ�36M��APB1)
//#if defined(STM32L1XX_MD)
//����ʱ��ѡ��Ϊ APB1���� APB1 Ϊ 32M(��SYSCLK==32Mʱ��1��Ƶ�õ�32M��APB1)
//#endif

//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//����ʹ�õ��Ƕ�ʱ�� 3!
static void TIM3_Int_Init(uint16_t arr,uint16_t psc)
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
  
  //���������������
  //���õ�STM32��ʱ���ĸ����ж�ʱ��������Щ������ֻҪ������ʱ������������һ���жϡ�
  //׼ȷ˵��ֻҪʹ�ܸ����ж�����λ��������Ӧһ�θ����жϡ���Ȼǰ�������NVICҲ�Ѿ����úá�
  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
  
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //����������ж�
  //�ж����ȼ� NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; //TIM3 �ж�
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//0; //��ռ���ȼ� 1 ��
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //�����ȼ� 1��
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ����ʹ��
  NVIC_Init(&NVIC_InitStructure); //�ܳ�ʼ�� NVIC �Ĵ���
  
//  TIM_Cmd(TIM3, ENABLE); //��ʹ�� TIM3
  
}

volatile void timer_task_schedule_func(void)
{
    
  for (unsigned char i=0; i < MAX_TASKS; ++i) //������������        
  {            
    if (tasks[i].slice_count !=0)  //��ʱ����ʹ��            
    { 
      tasks[i].slice_count -= TASK_TIEMR_BASE_MS;
      if (tasks[i].slice_count == 0) //ʱ��Ƭ����                 
      { 
        if(tasks[i].timer_handler)
        {
          tasks[i].is_run = true;//��λ  ��ʾ�������ִ�� 
          tasks[i].slice_count = tasks[i].reset_count; //���¼���ʱ��Ƭֵ��Ϊ�´���׼��                 
        } 
      }
    }        
  }

}
void start_timer_task_schedule()
{ 
  TIM_Cmd(TIM3, ENABLE); //��ʹ�� TIM3
}

void stop_timer_task_schedule()
{
  TIM_Cmd(TIM3, DISABLE); //��ʧ�� TIM3
}

void task_process()
{ 
    unsigned short i = 0;   
    for (i=0; i < MAX_TASKS; ++i) //������������     
    {         
      if (tasks[i].is_run == true) //�������ִ�У���ִ������         
      {   
        void *p = tasks[i].param;
        tasks[i].timer_handler(p);             
        tasks[i].is_run = false;//����־λ����        
      }    
    }

}


void task_init()
{

  task_reset();
#if defined(STM32L1XX_MD) 
  
   TIM3_Int_Init((TASK_TIEMR_BASE_MS*10-1),3199);//��ʱ���log,������������ȵĻ�׼ʱ��  
    //Tout= ((arr+1)*( psc+1))/Tclk=(99+1)*( 3199+1))/32=10ms
#else    
   TIM3_Int_Init(99,7199); //10Khz �ļ���Ƶ�ʣ������� 500 Ϊ 50ms
   //Tout= ((799+1)*( 7199+1))/72=500000us=80ms
#endif 
   
}
 
 