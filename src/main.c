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
#include "SMBus.h"
#include "sys.h"
#include "stdbool.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEM_CLOCK 72000000
#define DR_ADDRESS                  ((uint32_t)0x4001244C) //ADC1 DR�Ĵ�������ַ
#define TP1_OUT_PIN                  PAout(2)   
#define TP2_IN_PIN                   PAin(3)  
#define SMBALERT_IN_PIN              PAin(4)
#define U5NWP_OUT_PIN                PAout(5) 
#define NEQ_OUT_PIN                  PAout(6) 
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool ltc4015_powered = false;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;
__IO uint16_t ADCConvertedValue;     // ADCΪ12λģ��ת������ֻ��ADCConvertedValue�ĵ�12λ��Ч
/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
/* Private function prototypes -----------------------------------------------*/
void DC2039A_Config_Init(void);
void DC2039A_Run(void);
void DC2039A_Interface_Init(void);
void ADC_GPIO_Configuration(void);
uint16_t STM32_ADC_Read(void);

/* Private functions ---------------------------------------------------------*/
uint16_t STM32_ADC_Read(void)
{
  u16 ADCConvertedValueLocal, Precent = 0, Voltage = 0;
  
  ADCConvertedValueLocal = ADCConvertedValue;
  Precent = (ADCConvertedValueLocal*100/0x1000);	//����ٷֱ�
  Voltage = Precent*33;				        // 3.3V�ĵ�ƽ�������Ч��ƽ

  return Voltage;
}

void DC2039A_Interface_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_TEST_TP, ENABLE);

    /********************************************/
    /*  Configure TP1��TP2��nSMBALLERT��CAT5140_NWP��*/
    /********************************************/

    GPIO_InitStructure.GPIO_Pin = TEST_TP1_PIN | CAT5140_NWP_PIN | NEQ_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = nSMBALLERT_PIN | TEST_TP2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
    
    NEQ_OUT_PIN = 0;//EQ SET��0
    
    U5NWP_OUT_PIN = 1;//close write protect
    
    ADC_GPIO_Configuration();//EQ_ADC�ӿ�����  
    
    SMBus_Init();//����SMBus�ӿڣ���IIC��ȣ�ACK,NACK�Ļ���Ҫ���ǿ
    
    uint8_t cat5140_id = Get_CAT5140_ID();
    
    CAT5140_Optional_Handle();
  
}
void ADC_GPIO_Configuration(void)
{
   /* Config ADC-PC0 connect LTC4015-INT_VCC */
    
      /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
    
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;        //DMA��ʼ���ṹ������
    ADC_InitTypeDef ADC_InitStructure;        //ADC��ʼ���ṹ������
      
    
    /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		 //ʹ��DMAʱ��
      /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);	  //ʹ��ADC��GPIOCʱ��
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//����ADC��Ƶ����6
                                    //72M/6=12,ADC���ʱ�䲻�ܳ���14M
    
    /* Configure PC.00 (ADC Channel10) as analog input -------------------------*/
    //PC0 ��Ϊģ��ͨ��10��������                         
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //�ܽ�1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//����ģʽ
    GPIO_Init(GPIOC, &GPIO_InitStructure);     //GPIO��
   
    
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);		  //����DMA1�ĵ�һͨ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = DR_ADDRESS;		  //DMA��Ӧ���������ַ
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;   //�ڴ�洢����ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA��ת��ģʽΪSRCģʽ����������Ƶ��ڴ�
    DMA_InitStructure.DMA_BufferSize = 1;		   //DMA�����С��1��
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//����һ�����ݺ��豸��ַ��ֹ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;	//�رս���һ�����ݺ�Ŀ���ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //�����������ݿ��Ϊ16λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA�������ݳߴ磬HalfWord����Ϊ16λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //ת��ģʽ��ѭ������ģʽ��
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA���ȼ���
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2Mģʽ����
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		//������ת��ģʽ
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;		  //����ɨ��ģʽ
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //��������ת��ģʽ
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ADC�ⲿ���أ��ر�״̬
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //���뷽ʽ,ADCΪ12λ�У��Ҷ��뷽ʽ
    ADC_InitStructure.ADC_NbrOfChannel = 1;	 //����ͨ������1��
    ADC_Init(ADC1, &ADC_InitStructure);
    /* ADC1 regular channel10 configuration */ 
    //PC0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
                            //ADCͨ���飬 ��10��ͨ�� ����˳��1��ת��ʱ�� 
    
    
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);	  //ADC���ʹ��
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);  //����ADC1
    
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);	  //����У׼
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));  //�ȴ�����У׼���
    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);		//��ʼУ׼
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));	   //�ȴ�У׼���
    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//����ת����ʼ��ADCͨ��DMA��ʽ���ϵĸ���RAM����
      
}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void DC2039A_Config_Init(void)
{   
    uint16_t value =0;
    
    //Set min UVCL ;�����ѹ����13V�����ܿ�����繦��
    LTC4015_write_register(chip, LTC4015_VIN_UVCL_SETTING_BF, LTC4015_VIN_UVCL(13)); // Initialize UVCL Lo Limit to 13V
    
    //Set max VCHARGE_SETTING �����õ��ڵ�ص������ѹֵ
    LTC4015_write_register(chip, LTC4015_VCHARGE_SETTING_BF, LTC4015_VCHARGE_LIION(4.2)); //4.2v/cell
    
    
    //Set max ICHARGE_TARGE �����ó�������Ŀ��ֵ
    LTC4015_write_register(chip, LTC4015_ICHARGE_TARGET_BF, LTC4015_ICHARGE(1.5));//1.5A
    
     //set max input current  ������������������ֵ 
    LTC4015_write_register(chip, LTC4015_IIN_LIMIT_SETTING_BF, LTC4015_IINLIM(5.0)); // 5.0A, Initialize IIN Limit to 5A
   
    
    //Enables C/x charge termination��ʹ��
    LTC4015_write_register(chip, LTC4015_EN_C_OVER_X_TERM_BF, 1);
    
    //Enable QCount��ʹ�ܿ��׼���
    LTC4015_write_register(chip, LTC4015_EN_QCOUNT_BF, 1);

//    //ϵͳ��12ms��Ԥ��ʱ�䣬�����������Чʱ��������ʾ��ͨ��nSMBALERT   
//    //Enable measurement valid alert��ʹ�ܲ�����Ч��ʾ
//    LTC4015_write_register(chip, LTC4015_EN_MEAS_SYS_VALID_ALERT_BF, 1);
//    
//    //Enable measurement on��ǿ�ƴ򿪲�������
//    LTC4015_write_register(chip, LTC4015_FORCE_MEAS_SYS_ON_BF, 1);
    
    //
    //LTC4015_read_register(chip, LTC4015_VIN_UVCL_SETTING_BF, &value); 
    //���������ѹ���ޣ�10v
    LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_LIMIT, LTC4015_VIN_FORMAT(10)); // Initialize VIN Lo Limit to 10V
    //���������ѹ���޵͸澯����
    LTC4015_write_register(chip, LTC4015_EN_VIN_LO_ALERT_BF, 1); // Initialize VIN Lo Limit Alert to On
    return;
}

void DC2039A_Run(void)
{
    uint16_t value;
    uint8_t ara_address;       
    int result;
    float input_power_vcc = 0.0;
    float input_power_current = 0.0;
    float bat_charge_current = 0.0;
    float actual_charge_current = 0.0;
    float bat_chargr_vcc = 0.0;
    float die_temp = 0.0;
    
    bool ltc4015_powered_last = ltc4015_powered;

    // Read the INTVCC A/D value to know if part cycled power
    value = STM32_ADC_Read();
    ltc4015_powered = (value > 1700);//v��>1.7v
    
    // If power is cycled re-init.
    if((ltc4015_powered_last == false) && (ltc4015_powered == true)) DC2039A_Config_Init();
    
    //Read charger state(�˴�ֻ���������Դ���)
    LTC4015_read_register(chip, LTC4015_CHARGER_SUSPENDED_BF, &value);
    if(value == 1)//no vin
    {
      //ϵͳ��12ms��Ԥ��ʱ�䣬�����������Чʱ��������ʾ��ͨ��nSMBALERT   
      //Enable measurement valid alert��ʹ�ܲ�����Ч��ʾ
      LTC4015_write_register(chip, LTC4015_EN_MEAS_SYS_VALID_ALERT_BF, 1);
      
      //Enable measurement on��ǿ�ƴ򿪲�������
      LTC4015_write_register(chip, LTC4015_FORCE_MEAS_SYS_ON_BF, 1);
      
      int ms_delay_count =0;
      do
      {
        delay_ms(20);
        ms_delay_count++;
      }
      while(SMBALERT_IN_PIN | (ms_delay_count>3));
      if(!SMBALERT_IN_PIN)
      {
        // Clear the SMBAlert and get the address responding to the ARA.
            result = SMBus_ARA_Read(&ara_address, 0);
            
            // Read what caused the alerts and clear if LTC4015
            // so that it will be re-enabled.
            if((ara_address == LTC4015_ADDR_68) && (result == 0));
            {
                LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
                if((LTC4015_MEAS_SYS_VALID_ALERT_BF_MASK & value) !=0)//verify 
                {
                   //LTC4015_write_register(chip, LTC4015_MEAS_SYS_VALID_ALERT_BF, 0);  // Clear the Alert (this code only enabled one).
                  //close alert
                  //close meas
                  LTC4015_write_register(chip, LTC4015_EN_MEAS_SYS_VALID_ALERT_BF, 0); 
                  LTC4015_write_register(chip, LTC4015_FORCE_MEAS_SYS_ON_BF, 0);
                }
            }        
        }
    }
    
    //Read charge state
    LTC4015_read_register(chip, LTC4015_CONSTANT_CURRENT_BF, &value);
          
    //Read system status
    LTC4015_read_register(chip, LTC4015_CHARGER_ENABLED_BF, &value);
    
    //Read VIN
    LTC4015_read_register(chip, LTC4015_VIN_BF, &value);
    input_power_vcc = ((float)value)*1.648/1000;//v
    
     //Read IIN
    LTC4015_read_register(chip, LTC4015_IIN_BF, &value);
    input_power_current = ((float)value)*((1.46487/3)*0.001);//v
    
    //Read VBATSENSE/cellcount
    LTC4015_read_register(chip, LTC4015_VBAT_BF, &value);
    bat_chargr_vcc = ((float)value*192.264/1000000);
      
    //Read Bat charge current
    LTC4015_read_register(chip, LTC4015_IBAT_BF, &value);
    bat_charge_current = ((float)value)*((1.46487/4)*0.001);//A
    
    
    
    
    //Read actual charge current,v;
    LTC4015_read_register(chip, LTC4015_ICHARGE_DAC_BF, &value);
    actual_charge_current = (value+1)/4;//A
    
    //read die temperature
    LTC4015_read_register(chip, LTC4015_DIE_TEMP_BF, &value);
    die_temp = (float)(value-12010)/45.6;//��

    
    //����ؼ��֪ͨ
    // Show example of polling an alert and setting TP1 to reflect alert.
//    LTC4015_read_register(chip, LTC4015_EN_BAT_MISSING_FAULT_ALERT_BF, &value);
//    
//    TP1_OUT_PIN = value;
    
    // Show example of using SMBALert to detect alert when active.
    // Do not clear until TP2 is pulled low.
    //if(!TP2_IN_PIN && !SMBALERT_IN_PIN)
//    if(!SMBALERT_IN_PIN)
//    {
//        int8_t read_limit = 100;
//        do
//        {
//            uint8_t ara_address;       
//            int result;
//            
//            // Clear the SMBAlert and get the address responding to the ARA.
//            result = SMBus_ARA_Read(&ara_address, 0);
//            
//            // Read what caused the alerts and clear if LTC4015
//            // so that it will be re-enabled.
//            if((ara_address == LTC4015_ADDR_68) && (result == 0));
//            {
//                LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
//                LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_BF, 0);  // Clear the Alert (this code only enabled one).
//            }
//            read_limit--; // Don't allow to read forever.
//
//        } while(!SMBALERT_IN_PIN && !read_limit);       
//    }       
//    
    return;
}



/*******************************************************************************
* Function Name  : main.
* Description    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  delay_init(72);//��ʱ���ܳ�ʼ��
  //SysTick_Config(SYSTEM_CLOCK / 500);//2ms
  
  STM_EVAL_LEDInit(LED2);//����ԭ��ͼ���޸ĺ������LED�궨�弴��
  STM_EVAL_LEDInit(LED3);
  
  STM_EVAL_LEDOn(LED2);//�ߵ�ƽ������
  STM_EVAL_LEDOn(LED3);
  
  DC2039A_Interface_Init();
  
  Set_System();
  Set_USBClock();// ���� USB ʱ�ӣ�Ҳ���Ǵ� 72M ����Ƶ�õ� 48M �� USB ʱ�ӣ�1.5 ��Ƶ��
  USB_Interrupts_Config();// USB �����жϺ�USB �����ȼ����ݴ����ж�
  USB_Init();//���ڳ�ʼ�� USB��;
  
  //TIM2_Int_Init(20, 7199);//2ms
  TIM3_Int_Init(999,7199); //10Khz �ļ���Ƶ�ʣ������� 1000 Ϊ 100ms
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
        TIM_Cmd(TIM3, ENABLE);//������ʱ��3
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
        DC2039A_Run();//����
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
