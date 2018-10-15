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
#include "logger.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#if defined(STM32L1XX_MD)

#define SMBALERT_IN_PIN              PBin(14)
#define U5NWP_OUT_PIN                PBout(8) 

#else

#define SYSTEM_CLOCK 72000000
#define DR_ADDRESS                  ((uint32_t)0x4001244C) //ADC1 DR�Ĵ�������ַ
#define TP1_OUT_PIN                  PAout(2)   
#define TP2_IN_PIN                   PAin(3)  
#define SMBALERT_IN_PIN              PAin(4)
#define U5NWP_OUT_PIN                PAout(5) 
#define NEQ_OUT_PIN                  PAout(6) 

#endif
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool ltc4015_powered = false;
bool No_VIN_Flag = false;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;
__IO uint16_t ADCConvertedValue;     // ADCΪ12λģ��ת������ֻ��ADCConvertedValue�ĵ�12λ��Ч
LTC4015_charger_state_t charger_state;
LTC4015_charge_status_t charge_status;
LTC4015_system_status_t system_status;
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
  uint16_t ADCConvertedValueLocal, Precent = 0, Voltage = 0;
  
  ADCConvertedValueLocal = ADCConvertedValue;
  Precent = (ADCConvertedValueLocal*100/0x1000);	//����ٷֱ�
  Voltage = Precent*33;				        // 3.3V�ĵ�ƽ�������Ч��ƽ

  return Voltage;
}

void DC2039A_Interface_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;  
    
#if defined(STM32L1XX_MD)
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    /********************************************/
    /*  Configure nSMBALLERT,CAT5140_NWP*/
    /********************************************/
    
    GPIO_InitStructure.GPIO_Pin = CAT5140_NWP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* Push-pull or open drain */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* None, Pull-up or pull-down */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz; /* 400 KHz, 2, 10 or 40MHz */
    
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
     
    
    GPIO_InitStructure.GPIO_Pin = nSMBALLERT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* None, Pull-up or pull-down */
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
    
#else
  
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
    
#endif  
    
    
    //NEQ_OUT_PIN = 0;//EQ SET��0
    
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
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;

#if defined(STM32L1XX_MD)
    
     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	  //GPIOAʱ��
    /* Configure PA4 (ADC Channel 4) as analog input -------------------------*/
    //PA4 ��Ϊģ��ͨ��4��������                         
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //�ܽ�4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /* None, Pull-up or pull-down */
    GPIO_Init(GPIOA, &GPIO_InitStructure);     //GPIO��
    
    
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);		  //����DMA1�ĵ�һͨ��
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1->DR;//DR_ADDRESS;		  //DMA��Ӧ���������ַ
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
    
    
    
    /* Enable the HSI oscillator */
    RCC_HSICmd(ENABLE);
    /* Check that HSI oscillator is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
    {
    }

    /* Enable ADC1 clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    /* Common configuration *****************************************************/
    /* ADCCLK = HSI/1 */
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div1;
    ADC_CommonInit(&ADC_CommonInitStructure);
    /* ADC1 configuration */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    /* ADC1 regular channel 4 configuration */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_4Cycles);
    /* Enable the request after last transfer for DMA Circular mode */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
    /* Wait until the ADC1 is ready */
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET)
    {
    }
    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConv(ADC1);

    

#else    
    
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
#endif    
}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void DC2039A_Config_Init(void)
{   
    uint16_t value =0;
    
    log_info("init LTC4015 setting.");
    memset(&charger_state, 0x00, sizeof(LTC4015_charger_state_t));
    memset(&charge_status, 0x00, sizeof(LTC4015_charge_status_t));
    memset(&system_status, 0x00, sizeof(LTC4015_system_status_t));
    
    //Set min UVCL ;�����ѹ����13V�����ܿ�����繦��
    LTC4015_write_register(chip, LTC4015_VIN_UVCL_SETTING_BF, LTC4015_VIN_UVCL(13)); // Initialize UVCL Lo Limit to 13V
    
    //Set max VCHARGE_SETTING �����õ��ڵ�ص������ѹֵ
    LTC4015_write_register(chip, LTC4015_VCHARGE_SETTING_BF, LTC4015_VCHARGE_LIION(4.2)); //4.2v/cell
    
    
    //Set max ICHARGE_TARGE �����ó�������Ŀ��ֵ
    LTC4015_write_register(chip, LTC4015_ICHARGE_TARGET_BF, LTC4015_ICHARGE(4.0));//4.0A
    
     //set max input current  ������������������ֵ 
    LTC4015_write_register(chip, LTC4015_IIN_LIMIT_SETTING_BF, LTC4015_IINLIM(5.0)); // 5.0A, Initialize IIN Limit to 5A
   
//For Li-Ion batteries, if en_c_over_x_term=1 (0 by default), C/x charge termination occurs after the battery reaches the
//charge voltage level and IBAT drops below C_OVER_X_THRESHOLD. See the section C/x Termination.
//    
    //Enables C/x charge termination��ʹ��.�������������
    LTC4015_write_register(chip, LTC4015_EN_C_OVER_X_TERM_BF, true);
    
    //Enable QCount��ʹ�ܿ��׼���
    LTC4015_write_register(chip, LTC4015_EN_QCOUNT_BF, true);
    
    //����һ����صĳ�ʼ����������һ�γ�������У׼
    //set QCOUNT to the battery's initial state of charge:60%=16384+32768*0.6=36044
    //16384~49152:0%~100%;
    //battery capacity:5600mAh = 5.6 * 3600sec = 20160 C;
    //Qlsb=20160/65535 = 0.30762 C;
    //Qlsb=QCOUNT_PRESCALE_FACTOR/(Kqc*Rsnsb);Kqc=8333.33hz/v;Rsnsb=0.004��;
    //QCOUNT_PRESCALE_FACTOR = 0.30762*8333.33*0.004 = 10.254;
    //double QCOUNT_PRESCALE_FACTOR = QCOUNT_PRESCALE_FACTOR*2=20.5��21��
     //LTC4015_write_register(chip, LTC4015_QCOUNT_BF, 36044);//60%
    
     //���ÿ��׵ķŴ�����
    //set QCOUNT_PRESCALE_FACTOR
     LTC4015_write_register(chip, LTC4015_QCOUNT_PRESCALE_FACTOR_BF, 21);
     
     //���ÿ��׸ߵĸ澯����ֵ��49149
     LTC4015_write_register(chip, LTC4015_QCOUNT_HI_ALERT_LIMIT_BF, 49149);
     
     //ʹ�ܿ��׸߸澯����
     LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, true);
     
    //�������ĺ�ѹ���ʱ��Ϊ10���ӣ�600sec
     LTC4015_write_register(chip, LTC4015_MAX_CV_TIME_BF, LTC4015_MAX_CV_TIME_BF_PRESET__30MINS);
     
     //���������ʱ��Ϊ2Сʱ��7200sec
     LTC4015_write_register(chip, LTC4015_MAX_CHARGE_TIME_BF, LTC4015_MAX_CHARGE_TIME_BF_PRESET__2HOURS);
     
    //���������ѹ���ޣ�10v
    LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_LIMIT, LTC4015_VIN_FORMAT(10)); // Initialize VIN Lo Limit to 10V
    //���������ѹ���޵͸澯����
    LTC4015_write_register(chip, LTC4015_EN_VIN_LO_ALERT_BF, true); // Initialize VIN Lo Limit Alert to On
    
    LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, false);
    
    return;
}

void DC2039A_Run(void)
{
    uint16_t   value = 0;
    uint8_t    ara_address = 0; 
    uint8_t    read_limit = 100;
    int        result = 0;
    float input_power_vcc = 0.0;
    float vsys_vcc = 0.0;
    float input_power_current = 0.0;
    float bat_charge_current = 0.0;
    float actual_charge_current = 0.0;
    float batsens_cellcount_vcc = 0.0;
    float bat_filt_vcc = 0.0;
    float die_temp = 0.0;
    float Rntc_value = 0.0;
    float current_battery_capacity = 0.0;
    int   charging_times= 0;//sec
    
    static bool first_termination_flag= false;
    bool ltc4015_powered_last = ltc4015_powered;

    // Read the INTVCC A/D value to know if part cycled power
    value = STM32_ADC_Read();
    ltc4015_powered = (value > 1700);//v��>1.7v
    
    // If power is cycled re-init.
    if((ltc4015_powered_last == false) && (ltc4015_powered == true)) DC2039A_Config_Init();
    
    if(ltc4015_powered == false)
    {
      log_warning("LTC4015 is poweroff!");
      first_termination_flag = false;
      return;
    }
    
        
    //Read charger state
    LTC4015_read_register(chip, LTC4015_CHARGER_STATE, (uint16_t *)&charger_state);
    
    //Read charge status
    LTC4015_read_register(chip, LTC4015_CHARGE_STATUS, (uint16_t *)&charge_status);   
    
    //Read system status
    LTC4015_read_register(chip, LTC4015_SYSTEM_STATUS, (uint16_t *)&system_status);
      
    if(system_status.vin_gt_vbat == false)//no vin
    {
      log_warning("No Input Power!");
      No_VIN_Flag = true;
      //ϵͳ��12ms��Ԥ��ʱ�䣬�����������Чʱ��������ʾ��ͨ��nSMBALERT   
      //Enable measurement valid alert��ʹ�ܲ�����Ч��ʾ
      LTC4015_write_register(chip, LTC4015_EN_MEAS_SYS_VALID_ALERT_BF, true);
      
      //Enable measurement on��ǿ�ƴ򿪲�������
      LTC4015_write_register(chip, LTC4015_FORCE_MEAS_SYS_ON_BF, true);
      
      int ms_delay_count =0;
      do
      {
        delay_ms(200);
        ms_delay_count++;
      }
      while(ms_delay_count<5);
      if(!SMBALERT_IN_PIN)
      {
        do
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
                  LTC4015_write_register(chip, LTC4015_EN_MEAS_SYS_VALID_ALERT_BF, false); 
                  LTC4015_write_register(chip, LTC4015_FORCE_MEAS_SYS_ON_BF, false);
                }
                if((LTC4015_QCOUNT_LO_ALERT_BF_MASK & value) !=0)//���׵͸澯
                {  
                  log_warning("QCOUNT_LO_ALERT(no vin): true! reset charge and allow the battery to charge again."); 
                   //�رտ��׵͸澯����
                  LTC4015_write_register(chip, LTC4015_EN_QCOUNT_LOW_ALERT_BF, false);                      
                   //ʹ�ܿ��׸߸澯����
                  LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, true);
                  
                  //allow the battery to charge again.
                  LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, false); 
                                                      
                }
                if((LTC4015_VIN_LO_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).
                }
              }
            
             read_limit--;// Don't allow to read forever,100 times at most.
             
          } while(!SMBALERT_IN_PIN && !read_limit);    
       }
    }
    else//�������Դ
    {
      
        No_VIN_Flag = false;
        
        if((charger_state.c_over_x_term == true) || (charger_state.timer_term == true)) 
           //&&(first_termination_flag == false))//��һ�γ���
        {
          if(first_termination_flag == false)
          {
            LTC4015_write_register(chip, LTC4015_QCOUNT_BF, 49152);//overwritten to 49152:100%
            //LTC4015_read_register(chip, LTC4015_QCOUNT_BF, &value);
            first_termination_flag =true;
            log_info("termination_flag: true, reset qcount to 49152.");
          }
          else
          {
            log_warning("termination_flag: true(again!).");
          }
        }
        
        if(charger_state.max_charge_time_fault == 1)
        {
          log_warning("max_charge_time_fault: true.");
          //clear fault
          LTC4015_write_register(chip, LTC4015_MAX_CHARGE_TIME_BF, false);
          //reset max_charge_time
          LTC4015_write_register(chip, LTC4015_MAX_CHARGE_TIME_BF, LTC4015_MAX_CHARGE_TIME_BF_PRESET__2HOURS);
        }
        
         //Read charge time
        LTC4015_read_register(chip, LTC4015_MAX_CHARGE_TIMER_BF, &value);
        charging_times = value;//sec
        log_info("charging time: %d sec.", charging_times);
        
        if(!SMBALERT_IN_PIN)//�и澯
        {
            read_limit = 100;
            do
            {              
                // Clear the SMBAlert and get the address responding to the ARA.
                result = SMBus_ARA_Read(&ara_address, 0);
                
                // Read what caused the alerts and clear if LTC4015
                // so that it will be re-enabled.
                if((ara_address == LTC4015_ADDR_68) && (result == 0));
                {
                    LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
                    if((LTC4015_QCOUNT_HI_ALERT_BF_MASK & value) !=0)//verify���׸߸澯 
                    {
                      log_warning("QCOUNT_HI_ALERT: true! suspend charge.");
                      //LTC4015_read_register(chip, LTC4015_CHARGER_STATE, (uint16_t *)&charger_state);
                      //suspend charger.
                      LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, true); 
                      
                       //���ÿ��׵͵ĸ澯����ֵ��16384+32768*0.9995 = 49127.232.
                      LTC4015_write_register(chip, LTC4015_QCOUNT_LO_ALERT_LIMIT_BF, 49127);
                      
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, false); 
     
                      //ʹ�ܿ��׵͸澯����
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_LOW_ALERT_BF, true);  
                      
                      log_info("EN_QCOUNT_LOW_ALERT:    true. ");
                      log_info("EN_QCOUNT_HIGH_ALERT:   fasle. ");
                        
                    }
                    if((LTC4015_QCOUNT_LO_ALERT_BF_MASK & value) !=0)//���׵͸澯
                    {
                      log_warning("QCOUNT_LO_ALERT: true! reset charge and allow the battery to charge again.");                    
                       //�رտ��׵͸澯����
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_LOW_ALERT_BF, false);                      
                       //ʹ�ܿ��׸߸澯����
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, true);
                      
                      //allow the battery to charge again.
                      LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, false); 
                                          
                    }
                    if((LTC4015_VIN_LO_ALERT_BF_MASK & value) !=0)
                    {
                      LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).
                    }
                    //else
//                    {
//                      //need to clear the Alert                   
//                    }
                }
                read_limit--; // Don't allow to read forever,100 times at most.

            } while(!SMBALERT_IN_PIN && !read_limit);       
        }  
        
    }  
    
    //Read Qcount
    LTC4015_read_register(chip, LTC4015_QCOUNT_BF, &value);
    current_battery_capacity = ((float)(value-16384)/32768.0)*100;//%
    log_info("bat capacity: %f %%. ", current_battery_capacity);
    
      
     //Read NTC_RATIO
    //This algorithm is suitable for NTCS0402E3103FLT
    LTC4015_read_register(chip, LTC4015_NTC_RATIO_BF, &value);
    Rntc_value = ((float)(10000*value))/(21845.0-value);
    log_info("Rntc: %f R. ", Rntc_value);
    
    //Read actual charge current
    LTC4015_read_register(chip, LTC4015_ICHARGE_DAC_BF, &value);
    actual_charge_current = (value+1)/4;//A
    log_info("max charge current: %f A . ", actual_charge_current);
    
    if(No_VIN_Flag == false)
    {
      //Read VIN
      LTC4015_read_register(chip, LTC4015_VIN_BF, &value);
      input_power_vcc = ((float)value)*1.648/1000;//v
      log_info("VIN: %f V . ", input_power_vcc);
      
       //Read IIN
      LTC4015_read_register(chip, LTC4015_IIN_BF, &value);
      input_power_current = ((float)value)*((1.46487/3)*0.001);//v
      log_info("IIN: %f A . ", input_power_current);
      
      if(actual_charge_current != 0)
      {
        //Read Bat charge current
        LTC4015_read_register(chip, LTC4015_IBAT_BF, &value);
        bat_charge_current = ((float)value)*((1.46487/4)*0.001);//A
        log_info("IBAT: %f A . ", bat_charge_current);
      }
    }
    
    //Read VSYS
    LTC4015_read_register(chip, LTC4015_VSYS_BF, &value);
    vsys_vcc = ((float)value)*1.648/1000;//v
    log_info("VSYS: %f V . ", vsys_vcc);
    
    
  //When a Li-Ion battery charge cycle begins, the LTC4015
  //first determines if the battery is deeply discharged. If
  //the battery voltage is below 2.85V per cell (VBAT_FILT
  //below 14822) and BATSENS pin is above 2.6V then the
  //LTC4015 begins charging by applying a preconditioning
  //charge equal to ICHARGE_TARGET/10 (rounded down
  //to the next LSB), and reporting precharge = 1. When the
  //battery voltage exceeds 2.9V per cell (VBAT_FILT above
  //15082), the LTC4015 proceeds to the constant-current/
  //constant-voltage charging phase (cc_cv_charge = 1).
    
    //Read VBATSENSE/cellcount����ָbatsens pin
    LTC4015_read_register(chip, LTC4015_VBAT_BF, &value);
    batsens_cellcount_vcc = ((float)value*192.264/1000000);
    log_info("VBAT/CELL: %f V . ", batsens_cellcount_vcc);
    
    //Read battery voltage of filtered(per cell)
    LTC4015_read_register(chip, LTC4015_VBAT_FILT_BF, &value);
    bat_filt_vcc = ((float)value*192.264/1000000);
    log_info("VBAT_FILT/CELL: %f V . ", bat_filt_vcc);
    
    //read die temperature
    LTC4015_read_register(chip, LTC4015_DIE_TEMP_BF, &value);
    die_temp = (float)(value-12010)/45.6;//��
    log_info("DIE_TEMP: %f T. ", die_temp);

    LTC4015_read_register(chip, LTC4015_CHARGER_STATE, (uint16_t *)&charger_state); 
    if(charger_state.cc_cv_charge == 1)
    {
      if(charge_status.constant_current == 1)
      {
        log_info("Charger is in cc state by cc-cv");
      }
      else if(charge_status.constant_voltage == 1)
      {
        log_info("Charger is in cv state by cc-cv");
      }
      else
      {
        log_warning("the IIN or VIN is controled by setting");
      }
    }
    else if(charger_state.c_over_x_term == 1)
    {
      log_warning("Charger is in c_over_x_term state!");
    }
    else if(charger_state.charger_suspended == 1)
    {
        log_warning("Charger is in charger suspended state!");
    }
    else if(charger_state.bat_missing_fault == 1 || charger_state.bat_short_fault == 1)
    {
      log_warning("Charger is in battery error state!");
    }
    else
    {
      log_warning("Charger state  value_hex: 0x%4X", charger_state);
      log_warning("Charge  status value_hex: 0x%4X", charge_status);
      log_warning("System  status value_hex: 0x%4X", system_status);
    }
    
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
#if defined(STM32L1XX_MD)
   delay_init(32);
#else
   delay_init(72);//��ʱ���ܳ�ʼ��
#endif  
  logger_init();//logger ��ʼ��
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
  TIM3_Int_Init(99,7199); //10Khz �ļ���Ƶ�ʣ������� 500 Ϊ 50ms
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
        log_info("hello, LTC4105.");
      }
      CDC_Receive_DATA();
      /*Check to see if we have data yet */
      if (Receive_length  != 0)
      {
//        if (packet_sent == 1)
//          CDC_Send_DATA ((unsigned char*)Receive_Buffer,Receive_length);
       Receive_length = 0;
      }
      if(run_counts == 35*60000)
      {
        STM_EVAL_LEDToggle(LED2);
        run_counts = 0;
        if(timer3_run_flag)
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
