
/* Includes ------------------------------------------------------------------*/
#include "BCMP.h"
#include "DC2039A.h"
#include "stmflash.h"

extern const double Rntcbias;
extern const double Rp;//Ω
extern const double T2;//
extern const double Bx;//,NCP18XH103F0SRB
extern volatile g_alerts_t g_alerts;
extern bool save_charger_configuration(void*p, unsigned short length);

volatile bcmp_battery_info_brdcast_t g_bat_info;
//void bcmp_battery_info_reply()
//{
//  
//}

static void bcmp_tx( const bcmp_fragment_t * bcmp_p, unsigned char payload_len)
{
  

   /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
   
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_DATA_MSG_REQUEST;
   bnp_tx_frame.bnp_header.tx_number = DEFAULT_VALUE;//在bnp中做计算
   
   bnp_tx_frame.bnp_header.length = payload_len + sizeof(bcmp_p->bcmp_opcode);
   
   memcpy(&bnp_tx_frame.bnp_data.bnp_content_data_msg, bcmp_p, bnp_tx_frame.bnp_header.length);
   
   bnp_tx_frame.bnp_end.checksum = DEFAULT_VALUE;//非跨线程调用，安全。让所有校验在bnp里做
  
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
  /* send bnp frame*/	
  bnp_tx(&bnp_tx_frame);
  
  log_debug("BCMP [0x%4x] send Msg.", bcmp_p->bcmp_opcode);
}

void bcmp_opcode_not_support_reply(void)
{
  /*bcmp frame will be sent*/
  bcmp_fragment_t bcmp_frame;
  bcmp_frame.bcmp_opcode        = BCMP_REPLY;
  bcmp_frame.u8[0]              = UNSUPPORT_OPCODE;
  
  /*send bcmp frame*/
  bcmp_tx( &bcmp_frame, 1);
  
}

static void bcmp_battery_info_req_response(const bcmp_fragment_t *bcmp_rx_frame_p)
{
  
 /*bcmp frame will be sent*/
  bcmp_fragment_t bcmp_frame;
  bcmp_frame.bcmp_opcode = BCMP_REPLY | BATTERY_INFO;
   
  //强制转换
  bcmp_battery_info_reply_t *ptr =  (bcmp_battery_info_reply_t *)(bcmp_frame.u8);
  
  memcpy(&(ptr->detailed_info), (void*)&g_bat_info, sizeof(bcmp_battery_info_brdcast_t));
  
  ptr->result = SUCCESS_NO_PROBLEM;
  
  bcmp_tx(&bcmp_frame, sizeof(bcmp_battery_info_reply_t) +sizeof(ptr->result));

}

static void bcmp_alert_and_notice_config_req_response(const bcmp_fragment_t *bcmp_rx_frame_p)
{
  

}

static void bcmp_get_alert_info_req_response(const bcmp_fragment_t *bcmp_rx_frame_p)
{
  
  //unsigned char alert_count = 0;
  //unsigned short  value =0;
  
   /*bcmp frame will be sent*/
  bcmp_fragment_t bcmp_frame;
  bcmp_frame.bcmp_opcode = BCMP_REPLY | ALERT_INFO;
   
  //强制转换
  bcmp_alert_info_reply_t *ptr =  (bcmp_alert_info_reply_t *)(bcmp_frame.u8);

  //注意此处使用的是全局变量，多线程时需要加锁保护。
  ptr->result = g_alerts.result;

  ptr->number = g_alerts.number;

  memcpy(ptr->alert_obj, (void const*)g_alerts.alert_obj, sizeof(sizeof(bcmp_alert_detailed_info_t)*g_alerts.number));


//  int result = 0;
//              
//  LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
//  if(value != 0)
//  {  
//
//    unsigned short  limit =0;
//    unsigned short  cycle_count =0;
//      do
//      {
//        if((LTC4015_VIN_LO_ALERT_BF_MASK & value) !=0)//VIN_LO
//        {
//          LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).                 
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = VIN_LO_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_VIN_LO_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);
//          alert_count++;
//          value = (~LTC4015_VIN_LO_ALERT_BF_MASK)&value;
//        }           
//        else if((LTC4015_VIN_HI_ALERT_BF_MASK & value) !=0)//VIN_HI
//        {
//          LTC4015_write_register(chip, LTC4015_VIN_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = VIN_HI_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_VIN_HI_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);
//          alert_count++;    
//          value = (~LTC4015_VIN_HI_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_VBAT_HI_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_VBAT_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = VBAT_HI_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_VBAT_HI_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*192.264*1000);
//
//          alert_count++; 
//          value = (~LTC4015_VBAT_HI_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_VBAT_LO_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_VBAT_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = VBAT_LO_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_VBAT_LO_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*192.264*1000);
//
//
//          alert_count++; 
//          value = (~LTC4015_VBAT_LO_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_VSYS_LO_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_VSYS_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = VSYS_LO_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_VSYS_LO_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);
//
//
//          alert_count++; 
//          value = (~LTC4015_VSYS_LO_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_VSYS_HI_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_VSYS_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = VSYS_HI_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_VSYS_HI_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.648);
//
//
//          alert_count++; 
//          value = (~LTC4015_VSYS_HI_ALERT_BF_MASK)&value;
//        }
//
//        else if((LTC4015_IIN_HI_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_IIN_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          ptr->alert_obj[alert_count].id          = IIN_HI_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_IIN_HI_ALERT_LIMIT, &limit);   
//          ptr->alert_obj[alert_count].value       = (int16_t)round(((double)limit)*1.46487/4);
//
//          alert_count++; 
//          value = (~LTC4015_IIN_HI_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_IBAT_LO_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_IBAT_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//
//          ptr->alert_obj[alert_count].type        = LIMIT_ALERT;
//          g_alerts.alert_obj[alert_count].id          = IBAT_LO_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_IBAT_LO_ALERT_LIMIT, &limit);   
//          g_alerts.alert_obj[alert_count].value       = (int16_t)round(((double)((signed short)limit))*1.46487/4);
//
//
//          alert_count++; 
//          value = (~LTC4015_IBAT_LO_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_DIE_TEMP_HI_ALERT_BF_MASK & value) !=0)
//        {
//          LTC4015_write_register(chip, LTC4015_DIE_TEMP_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//
//          g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
//          g_alerts.alert_obj[alert_count].id          = DIE_TEMP_HI_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_DIE_TEMP_HI_ALERT_LIMIT, &limit);   
//          g_alerts.alert_obj[alert_count].value       = (signed short)((double)(value-12010)/45.6*100);
//
//          alert_count++; 
//          value = (~LTC4015_DIE_TEMP_HI_ALERT_BF_MASK)&value;
//        }
//        else if((LTC4015_NTC_RATIO_LO_ALERT_BF_MASK & value) !=0)
//        {
//          
//          LTC4015_write_register(chip, LTC4015_NTC_RATIO_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
//
//          g_alerts.alert_obj[alert_count].type        = LIMIT_ALERT;
//          g_alerts.alert_obj[alert_count].id          = BAT_TEMP_HI_ALERT_LIMIT; 
//
//          LTC4015_read_register(chip, LTC4015_NTC_RATIO_LO_ALERT_LIMIT, &limit);   
//          
//          
//          double ntc_temp =0.0;
//          double Rntc_value= 0.0;
//          Rntc_value = ((double)(Rntcbias*limit))/(21845.0-limit);
//          
//          //函数 log(x) 表示是以e为底的自然对数，即 ln(x)
//          ntc_temp = (1/(log(Rntc_value/Rp)/Bx + (1/T2)))-273.15+0.5;        
//          g_alerts.alert_obj[alert_count].value       = (signed short)(ntc_temp*100);
//
//          alert_count++; 
//          value = (~LTC4015_NTC_RATIO_LO_ALERT_BF_MASK)&value;
//          
//        }
//        cycle_count++;
//        if(cycle_count>=10)
//          break;
//  
//      }while(value!=0);
//    
//      ptr->result = SUCCESS_NO_PROBLEM;
//
//      ptr->number = alert_count;
//  
//  }
//  else
//  {
//  //need to clear the Alert    
//    
//    g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;
//    ptr->result = SUCCESS_NO_PROBLEM;
//    ptr->number = 0;
//  }
//  
  bcmp_tx(&bcmp_frame, 2 + ((ptr->number)*sizeof(bcmp_alert_detailed_info_t)));

}


//void read_charger_configuration()
//{
//  eeprom_read_nbyte();
//}


static void bcmp_config_charger_settings_req_response(const bcmp_fragment_t *bcmp_rx_frame_p)
{
  
   /*bcmp frame will be sent*/
  bcmp_fragment_t bcmp_frame;
  bcmp_frame.bcmp_opcode = BCMP_REPLY | CONFIG_CHARGER_SETTINGS;
  bcmp_frame.u8[0]       = SUCCESS_NO_PROBLEM; 
  /*send bcmp frame*/
  bcmp_tx( &bcmp_frame, 1);
  
  charger_settings_t * ptr = ( charger_settings_t * )bcmp_rx_frame_p->u8;//类型强制转换
  //save in eeprom
  save_charger_configuration(ptr, sizeof(charger_settings_t));
    
  //update setting of charger 
  DC2039A_Config_Param(ptr);

}



static const volatile BCMP_process_list_t bcmp_process_list[]=
{
  /*BCMP_REQEUST RESPONSE,         BCMP_REPLY,             BCMP_BROADCAST*/
  {     NULL                            },
  {bcmp_battery_info_req_response                },
  {bcmp_alert_and_notice_config_req_response     },
  {bcmp_get_alert_info_req_response                },
  {bcmp_config_charger_settings_req_response       },
  {              NULL                   },
    
};


//void bcmp_send_task(void *p)
//{
//   static unsigned int run_count = 0;
//   run_count++;
//   
//   log_debug("[bcmp_send_task] is running: %d", run_count);
//}


void bcmp_parse_func(const bcmp_fragment_t bcmp)
{
  
  if(((bcmp.bcmp_opcode & 0x000f) < sizeof(bcmp_process_list))
     && ((bcmp.bcmp_opcode & 0x000f) !=0))
  {
    log_debug("rx bcmp opcode:[0x%x]", bcmp.bcmp_opcode);
    bcmp_process_list[bcmp.bcmp_opcode & 0x000F].bcmp_rx_req_and_response_func(&bcmp);
  
  }
  else
  {
    bcmp_opcode_not_support_reply();
  }


}

extern volatile bnp_information_t bnp_information;
void bcmp_battery_info_broadcast_task(void *p)
{
  
  if(bnp_information.is_connected == true)
  {
     /*bcmp frame will be sent*/
    bcmp_fragment_t bcmp_frame;
    bcmp_frame.bcmp_opcode = BCMP_BRDCAST | BATTERY_INFO;
     
    //强制转换
    bcmp_battery_info_brdcast_t *ptr =  (bcmp_battery_info_brdcast_t *)(bcmp_frame.u8);
    
    memcpy(ptr, (void*)&g_bat_info, sizeof(bcmp_battery_info_brdcast_t));
 
    bcmp_tx(&bcmp_frame, sizeof(bcmp_battery_info_brdcast_t));
    
  }
  
}

  
void bcmp_init(void)
{
   memset((void*)&g_bat_info, 0x00, sizeof(bcmp_battery_info_brdcast_t));//clear buff
   
   g_bat_info.battery_state    = BAT_IDLE_SUSPEND;
   g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;
   
   set_timer_task(BCMP_BRDCAST_TASK, 20*TIME_BASE_500MS, true, bcmp_battery_info_broadcast_task, NULL);

}