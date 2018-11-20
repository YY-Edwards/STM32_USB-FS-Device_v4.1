
/* Includes ------------------------------------------------------------------*/
#include "BCMP.h"
#include "DC2039A.h"


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
  bcmp_frame.bcmp_opcode = BCMP_REPLY | ALERT_INFO;
   
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
  
  unsigned char alert_count = 0;
  unsigned short  value =0;
  
   /*bcmp frame will be sent*/
  bcmp_fragment_t bcmp_frame;
  bcmp_frame.bcmp_opcode = BCMP_REPLY | ALERT_INFO;
   
  //强制转换
  bcmp_alert_info_reply_t *ptr =  (bcmp_alert_info_reply_t *)(bcmp_frame.u8);

  if(!SMBALERT_IN_PIN)//有告警
    {
        g_bat_info.alert_identifier = ALERT_INFO_RESULT_HAPPEND;
        unsigned char read_limit = 10;
        uint8_t    ara_address = 0; 
        int        result = 0;
        do
        {              
            // Clear the SMBAlert and get the address responding to the ARA.
            result = SMBus_ARA_Read(&ara_address, 0);
            
            // Read what caused the alerts and clear if LTC4015
            // so that it will be re-enabled.
            if((ara_address == LTC4015_ADDR_68) && (result == 0));
            {
                LTC4015_read_register(chip, LTC4015_LIMIT_ALERTS, &value); // Read to see what caused alert
                unsigned short  limit =0;
                if((LTC4015_VIN_LO_ALERT_BF_MASK & value) !=0)//VIN_LO
                {
                  LTC4015_write_register(chip, LTC4015_VIN_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).                 
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = VIN_LO_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_VIN_LO_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*1.648);
                  alert_count++;
                }           
                if((LTC4015_VIN_HI_ALERT_BF_MASK & value) !=0)//VIN_HI
                {
                   LTC4015_write_register(chip, LTC4015_VIN_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                   ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                   ptr->alert_detailed_info[alert_count].id          = VIN_HI_ALERT_LIMIT; 

                   LTC4015_read_register(chip, LTC4015_VIN_HI_ALERT_LIMIT, &limit);   
                   ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*1.648);
                   alert_count++;                   
                }
                if((LTC4015_VBAT_HI_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_VBAT_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = VBAT_HI_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_VBAT_HI_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*192.264*1000);
                                   
                  alert_count++; 
                }
                  if((LTC4015_VBAT_LO_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_VBAT_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = VBAT_LO_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_VBAT_LO_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*192.264*1000);
                  
                  
                  alert_count++; 
                }
                  if((LTC4015_VSYS_LO_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_VSYS_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = VSYS_LO_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_VSYS_LO_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*1.648);
                  
                                 
                  alert_count++; 
                }
                  if((LTC4015_VSYS_HI_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_VSYS_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = VSYS_HI_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_VSYS_HI_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*1.648);
                  
                  
                  alert_count++; 
                }
                
                  if((LTC4015_IIN_HI_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_IIN_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = IIN_HI_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_IIN_HI_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*1.46487/4);
                  
                  alert_count++; 
                }
                  if((LTC4015_IBAT_LO_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_IBAT_LO_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = IBAT_LO_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_IBAT_LO_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)(((float)limit)*1.46487/4);
                  
                  
                  alert_count++; 
                }
                  if((LTC4015_DIE_TEMP_HI_ALERT_BF_MASK & value) !=0)
                {
                  LTC4015_write_register(chip, LTC4015_DIE_TEMP_HI_ALERT_BF, false);  // Clear the Alert (this code only enabled one).     
                  
                  ptr->alert_detailed_info[alert_count].type        = LIMIT_ALERT;
                  ptr->alert_detailed_info[alert_count].id          = DIE_TEMP_HI_ALERT_LIMIT; 

                  LTC4015_read_register(chip, LTC4015_DIE_TEMP_HI_ALERT_LIMIT, &limit);   
                  ptr->alert_detailed_info[alert_count].value       = (signed short)((float)(value-12010)/45.6*100);
                                 
                  alert_count++; 
                }
                else
                {
                  //need to clear the Alert                   
                }
            }
            read_limit--; // Don't allow to read forever,10 times at most.

        } while(!SMBALERT_IN_PIN && !read_limit); 
        
           
        ptr->result = ALERT_INFO_RESULT_HAPPEND;
        
        ptr->number = alert_count;
    } 
    else
    {
        g_bat_info.alert_identifier = ALERT_INFO_RESULT_NOTHING;
        ptr->result = ALERT_INFO_RESULT_NOTHING;
        ptr->number = 0;
    }
  
  
    bcmp_tx(&bcmp_frame, 2 + ((ptr->number)*sizeof(bcmp_alert_detailed_info_t)));

}


static const volatile BCMP_process_list_t bcmp_process_list[]=
{
  /*BCMP_REQEUST RESPONSE,         BCMP_REPLY,             BCMP_BROADCAST*/
  {     NULL                            },
  {bcmp_battery_info_req_response                },
  {bcmp_alert_and_notice_config_req_response     },
  {bcmp_get_alert_info_req_response                },
  {              NULL                   },
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
   
   g_bat_info.battery_state    = ALERT_INFO_RESULT_NOTHING;
   g_bat_info.alert_identifier = BAT_IDLE_SUSPEND;
   
   set_timer_task(BCMP_BRDCAST_TASK, 7*TIME_BASE_500MS, true, bcmp_battery_info_broadcast_task, NULL);

}