#include "DC2039A.h"
#include "stmflash.h"


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
bool ltc4015_powered = false;
bool No_VIN_Flag = false;
const double charger_efficency = 0.925;
//const double battery_total_capacity = 16.8;//Ah
const double Kqc=8333.33;//hz
const double Rntcbias=10000;//Ω
const double Rp = 10000;//Ω
const double T2 = (273.15+25.0);//
const double Bx = 3380;//,NCP18XH103F0SRB
const double Ka = 273.15;//

__IO uint16_t ADCConvertedValue;     // ADC为12位模数转换器，只有ADCConvertedValue的低12位有效
__IO LTC4015_charger_state_t charger_state;
__IO LTC4015_charge_status_t charge_status;
__IO LTC4015_system_status_t system_status;
extern volatile bcmp_battery_info_brdcast_t g_bat_info;

static void DC2039A_Run(void *);
static void charger_measure_data_func(void *);
static void charger_monitor_alert_func(void *);
void DC2039A_Init(void);
static void ADC_GPIO_Configuration(void);
static uint16_t INTVCC_ADC_Read(void);
void charger_monitor_task(void *);

//typedef bcmp_alert_info_reply_t g_alerts_t;
volatile g_alerts_t g_alerts;

/* Private functions ---------------------------------------------------------*/
uint16_t INTVCC_ADC_Read(void)
{
  uint16_t ADCConvertedValueLocal, Precent = 0, Voltage = 0;
  
  ADCConvertedValueLocal = ADCConvertedValue;
  Precent = (ADCConvertedValueLocal*100/0x1000);	//算出百分比
  Voltage = Precent*33;				        // 3.3V的电平，计算等效电平

  return Voltage;
}


extern void set_timer_task(unsigned char       timer_id, 
                            unsigned int        delay, 
                            unsigned char       rearm, 
                            handler              timehandler, 
                            void *               param );



void charger_monitor_task(void *p)
{
  
  //static int t = 0;
  //t++;
  //log_debug("[charger_monitor_task] is running :%d", t);
//  charger_settings_t settings;
//  settings.uvcl                 = 12000;//mv
//  settings.jeita                = 0;
//  settings.vcharge_cell         = 4200;//mv
//  settings.icharge_cell         = 5000;//ma
//  settings.iinlim               = 5000;//ma
//  settings.max_cv_time          = 1800;//sec
//  settings.max_charge_time      = 14400;//sec
//  settings.bat_capacity         = 16800;//mah
//  int idx =0;
//  settings.limits[idx].id         = VBAT_LO_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 3500;
//  idx++;
//  settings.limits[idx].id         = VBAT_HI_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 4200;
//  idx++;
//  settings.limits[idx].id         = VIN_LO_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 12000;
//   idx++;
//  settings.limits[idx].id         = VIN_HI_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 15000;
//   idx++;
//  settings.limits[idx].id         = VSYS_LO_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 10000;
//   idx++;
//  settings.limits[idx].id         = VSYS_HI_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 24000;
//   idx++;
//  settings.limits[idx].id         = IIN_HI_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 5500;
//   idx++;
//  settings.limits[idx].id         = IBAT_LO_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 1000;
//   idx++;
//  settings.limits[idx].id         = DIE_TEMP_HI_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 6000;
//   idx++;
//  settings.limits[idx].id         = BAT_TEMP_HI_ALERT_LIMIT;
//  settings.limits[idx].operation  = 0x01;
//  settings.limits[idx].value      = 6000;
//  
//  
//  save_charger_configuration(&settings);
  
  DC2039A_Run(p);

}
void DC2039A_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;  
    
#if defined(STM32L1XX_MD)
    
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_TEST_TP, ENABLE);
    /********************************************/
    /*  Configure nSMBALLERT,CAT5140_NWP*/
    /********************************************/
    
    GPIO_InitStructure.GPIO_Pin = CAT5140_NWP_PIN | LTC4015_DVCC_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /* Push-pull or open drain */
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; /* None, Pull-up or pull-down */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz; /* 400 KHz, 2, 10 or 40MHz */
    
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
     
    
    GPIO_InitStructure.GPIO_Pin = nSMBALLERT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /* None, Pull-up or pull-down */
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
    
#else
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_TEST_TP, ENABLE);

    /********************************************/
    /*  Configure TP1、TP2、nSMBALLERT、CAT5140_NWP、*/
    /********************************************/

    GPIO_InitStructure.GPIO_Pin = TEST_TP1_PIN | CAT5140_NWP_PIN | NEQ_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = nSMBALLERT_PIN | TEST_TP2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(TEST_TP, &GPIO_InitStructure);
    
#endif  
    
    
    //NEQ_OUT_PIN = 0;//EQ SET：0
    
//   while(SMBALERT_IN_PIN == 1)
//   {
//       delay_ms(100); 
//   }             
//   U5NWP_OUT_PIN   = 1;
//   U5NWP_OUT_PIN   = 0;
    
    U5NWP_OUT_PIN = 1;//close write protect
    
    ADC_GPIO_Configuration();//EQ_ADC接口配置  
    
    DVCC_OUT_PIN = 1;//output ICC dvcc
    
    SMBus_Init();//新增SMBus接口，与IIC相比，ACK,NACK的机制要求更强
    
    uint8_t cat5140_id = Get_CAT5140_ID();
    
    CAT5140_Optional_Handle();
    
    //3s
    set_timer_task(BATTERY_MONITOR_TASK, 18*TIME_BASE_500MS, true, charger_monitor_task, NULL);
  
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
    DMA_InitTypeDef DMA_InitStructure;        //DMA初始化结构体声明
    ADC_InitTypeDef ADC_InitStructure;        //ADC初始化结构体声明
    

#if defined(STM32L1XX_MD)
    
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	  //GPIOA时钟
    /* Configure PA4 (ADC Channel 4) as analog input -------------------------*/
    //PA4 作为模拟通道4输入引脚                         
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;     //管脚4
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; /* None, Pull-up or pull-down */
    GPIO_Init(GPIOA, &GPIO_InitStructure);     //GPIO组
    
      /* Enable DMA1 clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		 //使能DMA时钟
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);		  //开启DMA1的第一通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = DR_ADDRESS;		  //DMA对应的外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;   //内存存储基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA的转换模式为SRC模式，由外设搬移到内存
    DMA_InitStructure.DMA_BufferSize = 1;		   //DMA缓存大小，1个
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//接收一次数据后，设备地址禁止后移
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;	//关闭接收一次数据后，目标内存地址后移
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //定义外设数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA搬移数据尺寸，HalfWord就是为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //转换模式，循环缓存模式。
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA优先级高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2M模式禁用
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
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		 //使能DMA时钟
      /* Enable ADC1 and GPIOC clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);	  //使能ADC和GPIOC时钟
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//设置ADC分频因子6
                                    //72M/6=12,ADC最大时间不能超过14M
    
    /* Configure PC.00 (ADC Channel10) as analog input -------------------------*/
    //PC0 作为模拟通道10输入引脚                         
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;     //管脚1
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//输入模式
    GPIO_Init(GPIOC, &GPIO_InitStructure);     //GPIO组
   
    
    /* DMA1 channel1 configuration ----------------------------------------------*/
    DMA_DeInit(DMA1_Channel1);		  //开启DMA1的第一通道
    DMA_InitStructure.DMA_PeripheralBaseAddr = DR_ADDRESS;		  //DMA对应的外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;   //内存存储基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//DMA的转换模式为SRC模式，由外设搬移到内存
    DMA_InitStructure.DMA_BufferSize = 1;		   //DMA缓存大小，1个
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//接收一次数据后，设备地址禁止后移
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;	//关闭接收一次数据后，目标内存地址后移
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //定义外设数据宽度为16位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;  //DMA搬移数据尺寸，HalfWord就是为16位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;   //转换模式，循环缓存模式。
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;	//DMA优先级高
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		  //M2M模式禁用
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);          
    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    
    
    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		//独立的转换模式
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;		  //开启扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;   //开启连续转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ADC外部开关，关闭状态
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //对齐方式,ADC为12位中，右对齐方式
    ADC_InitStructure.ADC_NbrOfChannel = 1;	 //开启通道数，1个
    ADC_Init(ADC1, &ADC_InitStructure);
    /* ADC1 regular channel10 configuration */ 
    //PC0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
                            //ADC通道组， 第10个通道 采样顺序1，转换时间 
    
    
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);	  //ADC命令，使能
    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);  //开启ADC1
    
    /* Enable ADC1 reset calibaration register */   
    ADC_ResetCalibration(ADC1);	  //重新校准
    /* Check the end of ADC1 reset calibration register */
    while(ADC_GetResetCalibrationStatus(ADC1));  //等待重新校准完成
    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);		//开始校准
    /* Check the end of ADC1 calibration */
    while(ADC_GetCalibrationStatus(ADC1));	   //等待校准完成
    /* Start ADC1 Software Conversion */ 
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);	//连续转换开始，ADC通过DMA方式不断的更新RAM区。
#endif    
}
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void DC2039A_Config_Param(charger_settings_t *settings_ptr)
{   
    uint16_t value =0;
    
    log_info("reset LTC4015 settings.");
    
    memset((void*)&charger_state, 0x00, sizeof(LTC4015_charger_state_t));
    memset((void*)&charge_status, 0x00, sizeof(LTC4015_charge_status_t));
    memset((void*)&system_status, 0x00, sizeof(LTC4015_system_status_t));
    
    memset((void*)&g_bat_info, 0x00, sizeof(bcmp_battery_info_brdcast_t));//clear buff
    
    g_bat_info.bat_total_capacity = settings_ptr->bat_capacity;
    
    g_bat_info.battery_state    = BAT_IDLE_SUSPEND;
    g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;

    //Set min UVCL ;输入电压至少13V，才能开启充电功能
    LTC4015_write_register(chip, LTC4015_VIN_UVCL_SETTING_BF, LTC4015_VIN_UVCL(((double)(settings_ptr->uvcl)/1000))); // Initialize UVCL Lo Limit to 12V
    
    //Set max VCHARGE_SETTING ；设置单节电池的满电电压值,是否是31，验证一下
    //是一个目标值
    //影响恒压充电
    if(settings_ptr->jeita == 0x00)//disabled JEITA
    {
      LTC4015_write_register(chip, LTC4015_VCHARGE_SETTING_BF, LTC4015_VCHARGE_LIION(((double)(settings_ptr->vcharge_cell)/1000))); //4.2v/cell
         
      //Set max ICHARGE_TARGE ；设置充电电流的目标值
      LTC4015_write_register(chip, LTC4015_ICHARGE_TARGET_BF, LTC4015_ICHARGE(((double)(settings_ptr->icharge_cell)/1000)));//4.0A
    }
    
     //set max input current  ；设置输入电流的最大值目标值 
    LTC4015_write_register(chip, LTC4015_IIN_LIMIT_SETTING_BF, LTC4015_IINLIM(((double)(settings_ptr->iinlim)/1000))); // 5.0A, Initialize IIN Limit to 5A
   
//For Li-Ion batteries, if en_c_over_x_term=1 (0 by default), C/x charge termination occurs after the battery reaches the
//charge voltage level and IBAT drops below C_OVER_X_THRESHOLD. See the section C/x Termination.
//    
    //Enables C/x charge termination；使能.充满则挂起充电器
    LTC4015_write_register(chip, LTC4015_EN_C_OVER_X_TERM_BF, true);
    
    
    
    //Enable QCount；使能库伦计数
    LTC4015_write_register(chip, LTC4015_EN_QCOUNT_BF, true);
    
    //设置一个电池的初始容量，待第一次充满后再校准
    //set QCOUNT to the battery's initial state of charge:60%=16384+32768*0.6=36044
    //16384~49152:0%~100%;
    //battery capacity:5600mAh = 5.6 * 3600sec = 20160 C;
    //Qlsb=20160/65535 = 0.30762 C;
    //Qlsb=QCOUNT_PRESCALE_FACTOR/(Kqc*Rsnsb);Kqc=8333.33hz/v;Rsnsb=0.004Ω;
    //QCOUNT_PRESCALE_FACTOR = 0.30762*8333.33*0.004 = 10.254;
    //double QCOUNT_PRESCALE_FACTOR = QCOUNT_PRESCALE_FACTOR*2=20.5≈21；
     //LTC4015_write_register(chip, LTC4015_QCOUNT_BF, 36044);//60%
    
    //16.8*3600 = 60480 C;
    //Qlsb=60480/65535 = 0.92287 C;
    //Qlsb=QCOUNT_PRESCALE_FACTOR/(Kqc*Rsnsb);Kqc=8333.33hz/v;Rsnsb=0.004Ω;
    //QCOUNT_PRESCALE_FACTOR = 0.92287*8333.33*0.004 = 30.762;
    //double QCOUNT_PRESCALE_FACTOR = QCOUNT_PRESCALE_FACTOR*2=61.524≈62；
    
     //设置库伦的放大因子
    //set QCOUNT_PRESCALE_FACTOR
    unsigned int factor = (unsigned int)round(((double)settings_ptr->bat_capacity/1000)*3600/65535*Kqc*LTC4015_RSNSB*2);    
    LTC4015_write_register(chip, LTC4015_QCOUNT_PRESCALE_FACTOR_BF, factor);
     
         
    double vcc_per_bat = 0.0;
    LTC4015_read_register(chip, LTC4015_VBAT_FILT_BF, &value);
    vcc_per_bat = ((double)value*192.264/1000000);
    unsigned short acount_value_temp = 0 ;
    LTC4015_read_register(chip, LTC4015_QCOUNT_BF, &acount_value_temp);
    
    if((vcc_per_bat < 2.5) && (acount_value_temp <= 0x8000))  
     LTC4015_write_register(chip, LTC4015_QCOUNT_BF, 16384);//overwritten to 16384:0%
     
//     //设置库伦高的告警门限值：49149
//     LTC4015_write_register(chip, LTC4015_QCOUNT_HI_ALERT_LIMIT_BF, 49149);
//     
//     //使能库伦高告警功能
//     LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, true);
     
    //设置最大的恒压充电时间为30分钟：1800sec
     LTC4015_write_register(chip, LTC4015_MAX_CV_TIME_BF, LTC4015_MAX_CV_TIME_BF_PRESET__1HOUR);
     
     //设置最大充电时间为2小时：7200sec
     LTC4015_write_register(chip, LTC4015_MAX_CHARGE_TIME_BF, LTC4015_MAX_CHARGE_TIME_BF_PRESET__4HOURS);//大容量的充电时间更长
     
    
    
     /**********设置用户关心的告警类型:以下都是有符号数************/
     unsigned char limit_obj_count = LIMIT_CONFIG_COUNT;
     unsigned char idx =0;
     double dou_value = 0.0;
     double Rt= 0.0;//Ω
     do
     {
       switch(settings_ptr->limits[idx].id)
       {        
          case VIN_LO_ALERT_LIMIT:
                
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);
              //设置输入电压低门限；10v
                LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_LIMIT, LTC4015_VIN_FORMAT(dou_value)); // Initialize VIN Lo Limit to 13V
                //开启输入电压门限低告警功能
                LTC4015_write_register(chip, LTC4015_EN_VIN_LO_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize VIN Lo Limit Alert to On           
                    
            break;
            
          case VIN_HI_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);
                LTC4015_write_register(chip, LTC4015_VIN_HI_ALERT_LIMIT, LTC4015_VIN_FORMAT(dou_value)); // Initialize VIN Hi Limit to 15V
                LTC4015_write_register(chip, LTC4015_EN_VIN_HI_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize VIN Hi Limit Alert to On
                                
             break;
             
          case VBAT_HI_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);
                  //VBAT HI=> 22105*192.264uv = 4.25v
                LTC4015_write_register(chip, LTC4015_VBAT_HI_ALERT_LIMIT, LTC4015_VBAT_LITHIUM_FORMAT(dou_value)); // Initialize VBAT Hi Limit to 4.25V
                LTC4015_write_register(chip, LTC4015_EN_VBAT_HI_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize VBAT Hi Limit Alert to On    
                    
             break;
             
             
          case VBAT_LO_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);
                  //VBAT LO
                LTC4015_write_register(chip, LTC4015_VBAT_LO_ALERT_LIMIT, LTC4015_VBAT_LITHIUM_FORMAT(dou_value)); // Initialize VBAT Lo Limit to 3.5V
                LTC4015_write_register(chip, LTC4015_EN_VBAT_LO_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize VBAT Lo Limit Alert to On
                       
             break;
             
          case VSYS_HI_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);  
                //VSYS HI
                LTC4015_write_register(chip, LTC4015_VSYS_HI_ALERT_LIMIT, LTC4015_VSYS_FORMAT(dou_value)); // Initialize VSYS Hi Limit to 24V
                LTC4015_write_register(chip, LTC4015_EN_VSYS_HI_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize VSYS Hi Limit Alert to On

                        
            
             break;
             
          case VSYS_LO_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);  
                 //VSYS LO
                LTC4015_write_register(chip, LTC4015_VSYS_LO_ALERT_LIMIT, LTC4015_VSYS_FORMAT(dou_value)); // Initialize VSYS Lo Limit to 10V
                LTC4015_write_register(chip, LTC4015_EN_VSYS_LO_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize VSYS Lo Limit Alert to On
                
             break;
             
          case IIN_HI_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000);     
                //IIN HI
                LTC4015_write_register(chip, LTC4015_IIN_HI_ALERT_LIMIT, LTC4015_IIN_FORMAT(dou_value)); // Initialize IIN Hi Limit to 10A
                LTC4015_write_register(chip, LTC4015_EN_IIN_HI_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize IIN Hi Limit Alert to On
             
                      
            
             break;
             
          case IBAT_LO_ALERT_LIMIT:
                
                dou_value = ((double)(settings_ptr->limits[idx].value)/1000); 
                 //IBAT LO,赋值时也可以看着是放电电流，跟ISYS（评估电流）有待观察。
                LTC4015_write_register(chip, LTC4015_IBAT_LO_ALERT_LIMIT, LTC4015_IBAT_FORMAT(dou_value)); // Initialize IBAT Lo Limit to ±1A
                LTC4015_write_register(chip, LTC4015_EN_IBAT_LO_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize IBAT Lo Limit Alert to On
                                      
             break;
             
          case DIE_TEMP_HI_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/100);              
                //DIE HI
                LTC4015_write_register(chip, LTC4015_DIE_TEMP_HI_ALERT_LIMIT, LTC4015_DIE_TEMP_FORMAT(dou_value)); // Initialize DIE Hi Limit to 60℃
                LTC4015_write_register(chip, LTC4015_EN_DIE_TEMP_HI_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize DIE Hi Limit Alert to On
                                                 
             break;
             
          case BAT_TEMP_HI_ALERT_LIMIT:
            
                dou_value = ((double)(settings_ptr->limits[idx].value)/100);  
                Rt =Rp* pow(2.718,(Bx*(1/(Ka+dou_value)-1/T2)));//Rt=R*e^[B*(1/T1-1/T2)]
                dou_value = round(21845*Rt/(Rt+Rntcbias));// NTC_RATIO
                //BAT TEMP HI
                LTC4015_write_register(chip, LTC4015_NTC_RATIO_LO_ALERT_LIMIT, (uint16_t)dou_value); // Initialize DIE Hi Limit to 60℃
                LTC4015_write_register(chip, LTC4015_EN_NTC_RATIO_LO_ALERT_BF, settings_ptr->limits[idx].operation); // Initialize DIE Hi Limit Alert to On
                                      
             break;
             
             
          default:                      
            break;
               
       }
       
      idx++; 
      limit_obj_count--;
     }while(limit_obj_count>0);
         
    
    LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, false);
    
    return;
}

void read_charger_configuration(void*p, unsigned short length)
{
  eeprom_read_nbyte(0, (unsigned char* )p, length);
}

static void save_detailed_alert_info(void)
{
  unsigned char alert_count = 0;
  if(g_bat_info.alert_identifier == ALERT_INFO_RESULT_HAPPEND)//确实有告警
  {
      uint16_t   value = 0;
      LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
      if(value != 0)
      {  
          unsigned short  limit =0;
          unsigned short  cycle_count =0;
          do
          {
            if((LTC4015_VIN_LO_ALERT_BF_MASK & value) !=0)//VIN_LO
            {
              LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).                 
              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = VIN_LO_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_VIN_LO_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);
              alert_count++;
              value = (~LTC4015_VIN_LO_ALERT_BF_MASK)&value;
            }           
            else if((LTC4015_VIN_HI_ALERT_BF_MASK & value) !=0)//VIN_HI
            {
              LTC4015_write_register(chip, LTC4015_VIN_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = VIN_HI_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_VIN_HI_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);
              alert_count++;    
              value = (~LTC4015_VIN_HI_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_VBAT_HI_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_VBAT_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = VBAT_HI_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_VBAT_HI_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*192.264*1000);

              alert_count++; 
              value = (~LTC4015_VBAT_HI_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_VBAT_LO_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_VBAT_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = VBAT_LO_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_VBAT_LO_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*192.264*1000);


              alert_count++; 
              value = (~LTC4015_VBAT_LO_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_VSYS_LO_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_VSYS_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     

              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = VSYS_LO_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_VSYS_LO_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);


              alert_count++; 
              value = (~LTC4015_VSYS_LO_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_VSYS_HI_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_VSYS_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     

              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = VSYS_HI_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_VSYS_HI_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);


              alert_count++; 
              value = (~LTC4015_VSYS_HI_ALERT_BF_MASK)&value;
            }

            else if((LTC4015_IIN_HI_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_IIN_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     

              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = IIN_HI_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_IIN_HI_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.46487/4);

              alert_count++; 
              value = (~LTC4015_IIN_HI_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_IBAT_LO_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_IBAT_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     

              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = IBAT_LO_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_IBAT_LO_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)((signed short)limit))*1.46487/4);


              alert_count++; 
              value = (~LTC4015_IBAT_LO_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_DIE_TEMP_HI_ALERT_BF_MASK & value) !=0)
            {
              LTC4015_write_register(chip, LTC4015_DIE_TEMP_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     

              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = DIE_TEMP_HI_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_DIE_TEMP_HI_ALERT_LIMIT, &limit);   
              g_alerts.alert_obj[alert_count].value       = (signed short)((double)(value-12010)/45.6*100);

              alert_count++; 
              value = (~LTC4015_DIE_TEMP_HI_ALERT_BF_MASK)&value;
            }
            else if((LTC4015_NTC_RATIO_LO_ALERT_BF_MASK & value) !=0)
            {
              
              LTC4015_write_register(chip, LTC4015_NTC_RATIO_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     

              g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
              g_alerts.alert_obj[alert_count].id          = BAT_TEMP_HI_ALERT_LIMIT; 

              LTC4015_read_register(chip, LTC4015_NTC_RATIO_LO_ALERT_LIMIT, &limit);   
              
              
              double ntc_temp =0.0;
              double Rntc_value= 0.0;
              Rntc_value = ((double)(Rntcbias*limit))/(21845.0-limit);
              
              //函数 log(x) 表示是以e为底的自然对数，即 ln(x)
              ntc_temp = (1/(log(Rntc_value/Rp)/Bx + (1/T2)))-273.15+0.5;        
              g_alerts.alert_obj[alert_count].value       = (signed short)(ntc_temp*100);

              alert_count++; 
              value = (~LTC4015_NTC_RATIO_LO_ALERT_BF_MASK)&value;
              
            }
            cycle_count++;
            if(cycle_count>=10)
              break;
      
          }while(value!=0);      
          
          g_alerts.result = SUCCESS_NO_PROBLEM;

          g_alerts.number = alert_count;
      }
  
  }
   else
  {
  //need to clear the Alert    
    
    //g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;
    g_alerts.result = SUCCESS_NO_PROBLEM;
    g_alerts.number = 0;
    memset((void*)g_alerts.alert_obj, 0x00, sizeof(MAX_ALERT_COUNT * sizeof(bcmp_alert_detailed_info_t)));
  }
  
  
}
static void charger_monitor_alert_func(void *p)
{
    uint16_t   value = 0;
    uint16_t   alerts_bits = 0;
    uint8_t    ara_address = 0; 
    uint8_t    read_limit = 10;
    int        result = 0;
       
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
      //系统有12ms的预热时间，当测量结果有效时，会有提示，通过nSMBALERT   
      //Enable measurement valid alert；使能测量有效提示
      LTC4015_write_register(chip, LTC4015_EN_MEAS_SYS_VALID_ALERT_BF, true);   
      //Enable measurement on；强制打开测量功能
      LTC4015_write_register(chip, LTC4015_FORCE_MEAS_SYS_ON_BF, true);
      
      int ms_delay_count =0;
      do
      {
        delay_ms(20);
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
                  
                  alerts_bits = ((~LTC4015_MEAS_SYS_VALID_ALERT_BF_MASK)&value);
                }
                
                if((LTC4015_QCOUNT_LO_ALERT_BF_MASK & value) !=0)//库伦低告警
                {  
                  log_warning("QCOUNT_LO_ALERT(no vin): true! reset charge and  charging again."); 
                   //关闭库伦低告警功能
                  LTC4015_write_register(chip, LTC4015_EN_QCOUNT_LOW_ALERT_BF, false);                      
                  
                  //allow the battery to charge again.
                  LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, false); 
                  
                  alerts_bits = ((~LTC4015_QCOUNT_LO_ALERT_BF_MASK)&value);
                                                      
                }
                
                if(alerts_bits != 0)
                {
                  g_bat_info.alert_identifier = ALERT_INFO_RESULT_HAPPEND;
                  log_warning("SMBAlerts bits:[0x%x]", alerts_bits);
                }
              }
            
             read_limit--;// Don't allow to read forever,100 times at most.
             
          } while(!SMBALERT_IN_PIN && !read_limit);    
       }
      else
      {
        g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;
      }
    }
    else//有输入电源
    {
      
        No_VIN_Flag = false;
        
        if((charger_state.c_over_x_term == true) || (charger_state.timer_term == true)) 
        {
                     
           LTC4015_write_register(chip, LTC4015_QCOUNT_BF, 49153);//overwritten to 49152:100%
            //LTC4015_read_register(chip, LTC4015_QCOUNT_BF, &value);
           //first_termination_flag =true;
           
                     //49152
           LTC4015_write_register(chip, LTC4015_QCOUNT_HI_ALERT_LIMIT_BF, 49152);
           
           //使能库伦高告警功能
           LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, true);
           log_warning("termination_flag: true, reset qcount to 49153.");

        }
             
        if(!SMBALERT_IN_PIN)//有告警
        {
            read_limit = 10;
            do
            {              
                // Clear the SMBAlert and get the address responding to the ARA.
                result = SMBus_ARA_Read(&ara_address, 0);
                
                // Read what caused the alerts and clear if LTC4015
                // so that it will be re-enabled.
                if((ara_address == LTC4015_ADDR_68) && (result == 0));
                {
                    LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
                    if((LTC4015_QCOUNT_HI_ALERT_BF_MASK & value) !=0)//verify库伦高告警 
                    {
                      log_warning("QCOUNT_HI_ALERT(49152): true! suspend charge.");
                      //LTC4015_read_register(chip, LTC4015_CHARGER_STATE, (uint16_t *)&charger_state);
                      //suspend charger.
                      LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, true); 
                      
                       //设置库伦低的告警门限值：16384+32768*0.9995 = 49135.232.
                      LTC4015_write_register(chip, LTC4015_QCOUNT_LO_ALERT_LIMIT_BF, 49135);
                      
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, false); 
     
                      //使能库伦低告警功能
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_LOW_ALERT_BF, true);  
                      
                      log_info("EN_QCOUNT_LOW_ALERT(49135):    true. ");
                      log_info("EN_QCOUNT_HIGH_ALERT:   fasle. ");
                      
                      alerts_bits = ((~LTC4015_QCOUNT_HI_ALERT_BF_MASK)&value);
                        
                    }
                                     
                    if((LTC4015_QCOUNT_LO_ALERT_BF_MASK & value) !=0)//库伦低告警
                    {
                      log_warning("QCOUNT_LO_ALERT: true! reset charge and allow the battery to charge again.");                    
                       //关闭库伦低告警功能
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_LOW_ALERT_BF, false);                      
                       //使能库伦高告警功能
                      LTC4015_write_register(chip, LTC4015_EN_QCOUNT_HIGH_ALERT_BF, true);
                      
                      //allow the battery to charge again.
                      LTC4015_write_register(chip, LTC4015_SUSPEND_CHARGER_BF, false); 
                      
                      alerts_bits = ((~LTC4015_QCOUNT_LO_ALERT_BF_MASK)&value);                    
                    }
                    
                     if(alerts_bits != 0)
                    {
                      g_bat_info.alert_identifier = ALERT_INFO_RESULT_HAPPEND;
                      log_warning("SMBAlerts bits:[0x%x]", alerts_bits);
                    }       
                    
                    
                }
                read_limit--; // Don't allow to read forever,100 times at most.

            } while(!SMBALERT_IN_PIN && !read_limit);       
        }
        else
        {
          
          g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;
        
        }
       
        if(charger_state.max_charge_time_fault == 1)
        {
          log_warning("max_charge_time_fault: true.");
          //clear fault
          LTC4015_write_register(chip, LTC4015_MAX_CHARGE_TIME_BF, false);
          //reset max_charge_time
          LTC4015_write_register(chip, LTC4015_MAX_CHARGE_TIME_BF, LTC4015_MAX_CHARGE_TIME_BF_PRESET__4HOURS);
        }
        
    }
                             
    save_detailed_alert_info();//如果有告警，则保存到全局变量
    
    LTC4015_read_register(chip, LTC4015_CHARGER_STATE, (uint16_t *)&charger_state); 
    if((charger_state.cc_cv_charge == 1) || (charger_state.precharge == 1))
    {
      if(charge_status.constant_current == 1)
      {
        log_info("Charger is in cc state of cc-cv");
      }
      else if(charge_status.constant_voltage == 1)
      {
        log_info("Charger is in cv state of cc-cv");
      }
      else if(charger_state.precharge)
      {
        log_info("Charger is in precharge state ");
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
      g_bat_info.battery_state = BAT_ABNORMAL;
    }
    else
    {
      log_warning("Charger state  value_hex: 0x%2X", charger_state);
      log_warning("Charge  status value_hex: 0x%2X", charge_status);
      log_warning("System  status value_hex: 0x%2X", system_status);
    }
    
    
    if(g_bat_info.ICHARGER > 0)
    {
      g_bat_info.battery_state = BAT_CHARGING;
    }
    else//<0
    {
       if(abs(g_bat_info.ICHARGER) >= 100)//-100ma
      {
        g_bat_info.battery_state = BAT_DISCHARGING;
      }
      else
      {
        g_bat_info.battery_state = BAT_IDLE_SUSPEND;
      }
    }
    
    //log_info("\r\n");
    return;
  
}



static void charger_measure_data_func(void *p)
{
  
  
    uint16_t   value = 0;

    
    double input_power_vcc = 0.0;
    double vsys_vcc = 0.0;
    double input_power_current = 0.0;
    double actual_bat_current = 0.0;//实际的电池电流（正值是充电电流，负值是放电电流）
    double max_bat_charge_current = 0.0;//最大的充电电流
    double batsens_cellcount_vcc = 0.0;//电池电压
    double bat_filt_vcc = 0.0;//过滤后的电池电压
    double bat_charge_vcc = 0.0;//电池的充电电压
    double die_temp = 0.0;
    double Rntc_value = 0.0;
    double current_battery_capacity = 0.0;
    int    charging_times= 0;//sec  
    
    //Read charge time
    LTC4015_read_register(chip, LTC4015_MAX_CHARGE_TIMER_BF, &value);
    charging_times = value;//sec
    log_info("charging time: %d sec.", charging_times);
    
   //Read Qcount
    double capacity_percent =0.0;
    LTC4015_read_register(chip, LTC4015_QCOUNT_BF, &value);
    capacity_percent = ((double)(value-16384)/32768.0)*100;//%
    current_battery_capacity = capacity_percent/100*((double)(g_bat_info.bat_total_capacity)/1000);   
       
    g_bat_info.bat_currently_capacity = (uint32_t)round(current_battery_capacity*1000);
    log_info("bat capacity: %d, %f Ah, %f %%. ", value, current_battery_capacity, capacity_percent);
    
      
     //Read NTC_RATIO
    //This algorithm is suitable for NTCS0402E3103FLT
    double ntc_temp =0.0;
    LTC4015_read_register(chip, LTC4015_NTC_RATIO_BF, &value);
    Rntc_value = ((double)(Rntcbias*value))/(21845.0-value);
    
    //函数 log(x) 表示是以e为底的自然对数，即 ln(x)
    ntc_temp = (1/(log(Rntc_value/Rp)/Bx + (1/T2)))-273.15+0.5;
    
    g_bat_info.NTC = (int16_t)round(ntc_temp*100);
    log_info("Tntc: %f T.", ntc_temp);
    
    //Read max bat charge current
    LTC4015_read_register(chip, LTC4015_ICHARGE_DAC_BF, &value);
    max_bat_charge_current = (value+1)/4;//A
    //log_info("max charge current: %f A . ", max_bat_charge_current);
    
    //if(No_VIN_Flag == false)
    {
      //Read VIN
      LTC4015_read_register(chip, LTC4015_VIN_BF, &value);
      input_power_vcc = ((double)(signed short)value)*1.648/1000;//v
      g_bat_info.VIN = (int16_t)round(input_power_vcc*1000);
      log_info("VIN: %f V . ", input_power_vcc);
      
       //Read IIN
      LTC4015_read_register(chip, LTC4015_IIN_BF, &value);
      input_power_current = ((double)(signed short)value)*((1.46487/3)*0.001);//A
      g_bat_info.IIN = (int16_t)round(input_power_current*1000);     
      log_info("IIN: %f A . ", input_power_current);
      
      //if(max_bat_charge_current != 0)
      {
        //Read Bat charge current
        LTC4015_read_register(chip, LTC4015_IBAT_BF, &value);
        actual_bat_current = ((double)((signed short)value)*((1.46487/4)*0.001));//A     
        //有方向的
        g_bat_info.ICHARGER = (int16_t)round(actual_bat_current*1000);
        log_info("IBAT: %f A . ", actual_bat_current);
      }
     
    }
    
    //Read VSYS
    LTC4015_read_register(chip, LTC4015_VSYS_BF, &value);
    vsys_vcc = ((double)(signed short)value)*1.648/1000;//v
    g_bat_info.VSYS = (int16_t)round(vsys_vcc*1000);
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
    
     //read cell count as set by CELLS pins 
    int  bat_cell_count= 0;
    int  bat_chemistry= 0;
    LTC4015_read_register(chip, LTC4015_CHEM_BF, &value);    
    bat_chemistry = value;
    LTC4015_read_register(chip, LTC4015_CELL_COUNT_PINS_BF, &value);    
    bat_cell_count = value;
    //log_info("BAT TYPE,CELL: %d, %d ", bat_chemistry, bat_cell_count);
    
    //Read VBATSENSE/cellcount：特指batsens pin
    LTC4015_read_register(chip, LTC4015_VBAT_BF, &value);
    batsens_cellcount_vcc = ((double)(signed short)value*192.264/1000000);
    //log_info("VBAT/CELL: %f V . ", batsens_cellcount_vcc);
    
    //Read battery voltage of filtered(per cell)
    LTC4015_read_register(chip, LTC4015_VBAT_FILT_BF, &value);
    bat_filt_vcc = ((double)(signed short)value*192.264/1000000);
    log_info("VBAT_FILT/CELL: %f V . ", bat_filt_vcc); 

    //Read charge vcc of bat
    //初始化设置的值。根据电池化学类型、充电器状态、温度、的不同而不同。
    LTC4015_read_register(chip, LTC4015_VCHARGE_DAC_BF, &value);
    bat_charge_vcc = ((double)(signed short)value/80.0+3.8125);
    g_bat_info.VCHARGER = (int16_t)round(bat_charge_vcc * 1000 * bat_cell_count);//mv
    //log_info("BAT_CHARGE_VCC/CELL: %f V . ", bat_charge_vcc); 
    
    //calculate ISYS(estimated)
    double i_sys = 0.0;
    i_sys = (((input_power_vcc*input_power_current) - ((bat_filt_vcc*bat_cell_count*actual_bat_current)/charger_efficency))/vsys_vcc);
    log_info("ISYS: %f A. ", i_sys);
    i_sys *=1000;  //放大：ma
    g_bat_info.ISYS = (int16_t)round(i_sys);
    
        
    //read die temperature
    LTC4015_read_register(chip, LTC4015_DIE_TEMP_BF, &value);
    die_temp = (double)((signed short)value-12010)/45.6;//℃
    g_bat_info.DIE = (int16_t)round(die_temp*100);
    log_info("DIE_TEMP: %f T. ", die_temp);
    
    g_bat_info.discharge_time = 
      (unsigned int)round((current_battery_capacity/(i_sys/1000))*60);
    
    log_info("discharge_time: %d minutes. ", g_bat_info.discharge_time);
    
    double e_i_charge = 0;
    if(actual_bat_current < 0.8)
    {
      e_i_charge = 0.8;//A
    }
    else
    {
      e_i_charge = actual_bat_current;
    }
    g_bat_info.remained_charge_time = 
      (unsigned int)round(((double)(g_bat_info.bat_total_capacity)/1000 - current_battery_capacity)/e_i_charge*60);
    
    log_info("remained_charge_time: %d minutes. ", g_bat_info.remained_charge_time);

}


void DC2039A_Run(void *p)
{
    uint16_t   value = 0;
    
    //static bool first_termination_flag= false;
    bool ltc4015_powered_last = ltc4015_powered;

    // Read the INTVCC A/D value to know if part cycled power
    value = INTVCC_ADC_Read();
    ltc4015_powered = (value > 2500);//v？>2.5v
    
    // If power is cycled re-init.
    if((ltc4015_powered_last == false) && (ltc4015_powered == true))
    {
      charger_settings_t settings;
      memset(&settings, 0x00, sizeof(charger_settings_t));
      read_charger_configuration((void*)&settings, sizeof(charger_settings_t));
      //read_charger_configuration((void*)&settings, sizeof(charger_settings_t));
      DC2039A_Config_Param(&settings);
    }
    
    if(ltc4015_powered == false)//充电器未工作
    {
      log_warning("LTC4015 is poweroff!");
      g_bat_info.battery_state = BAT_ABNORMAL;
      return;
    }
    else  
    {     
      charger_measure_data_func(NULL);//1.测量数据
      
      charger_monitor_alert_func(NULL);//2.根据数据，监控告警信息
    
    }
}










