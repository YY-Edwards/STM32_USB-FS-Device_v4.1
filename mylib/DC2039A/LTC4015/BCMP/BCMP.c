
/* Includes ------------------------------------------------------------------*/
#include "BCMP.h"


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
  
  ptr->result = SUCCESS_NO_PROBLEM;
  //ptr->detailed_info;
  
  bcmp_tx(&bcmp_frame, sizeof(bcmp_battery_info_reply_t));

}

static void bcmp_alert_and_notice_config_req_response(const bcmp_fragment_t *bcmp_rx_frame_p)
{
  

}

static void bcmp_get_alert_info_req_response(const bcmp_fragment_t *bcmp_rx_frame_p)
{
  
   /*bcmp frame will be sent*/
  bcmp_fragment_t bcmp_frame;
  bcmp_frame.bcmp_opcode = BCMP_REPLY | ALERT_INFO;
   
  //强制转换
  bcmp_alert_info_reply_t *ptr =  (bcmp_alert_info_reply_t *)(bcmp_frame.u8);
  if(1)
  {
    ptr->result = ALERT_INFO_RESULT_HAPPEND;
    
    ptr->number = 3;
    
    ptr->alert_detailed_info[0].type        = LIMIT_ALERT;
    ptr->alert_detailed_info[0].id          = VIN_LO_ALERT_LIMIT;
    ptr->alert_detailed_info[0].value       = 0.0;
    
  }
  else
  {
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

  
void bcmp_init(void)
{
  

}