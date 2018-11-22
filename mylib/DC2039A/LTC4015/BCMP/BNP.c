
/* Includes ------------------------------------------------------------------*/
#include "BNP.h"
#include "BCMP.h"
#include "PHY_Slip.h"

volatile bnp_information_t bnp_information;
static volatile unsigned short server_transaction_id = 1;
volatile RingQueue_t bnp_rx_queue_ptr = NULL;
volatile RingQueue_t bnp_tx_queue_ptr = NULL;
volatile bool bnp_rx_response_flag = false;

/*Defines the callback function is used to handle BCMP*/
void ( *bcmp_analyse_callback_func)(const bnp_content_data_msg_t) = NULL;

void bnp_set_bcmp_analyse_callback( void ( *func)(const bnp_content_data_msg_t))
{
  
  bcmp_analyse_callback_func = func;

}
  

static unsigned short check_sum (const bnp_fragment_t * bnp_p)
{
	
	unsigned short  sumScratch = 0;
        
        sumScratch += bnp_p->bnp_header.opcode;
        
        sumScratch += (bnp_p->bnp_header.tx_number>>8 & 0xFF);
        sumScratch += (bnp_p->bnp_header.tx_number & 0xFF);
        sumScratch += (bnp_p->bnp_header.length>>8 & 0xFF);
        sumScratch += (bnp_p->bnp_header.length & 0xFF);
        
//        sumScratch = 
//          bnp_p->bnp_header.opcode 
//            + bnp_p->bnp_header.tx_number
//              + bnp_p->bnp_header.length;
        
        unsigned short len = bnp_p->bnp_header.length;

        for(unsigned short i =0; i<len; i++)
        {
          sumScratch += bnp_p->bnp_data.u8[i];
        }
                   
        return sumScratch;
	
}

//send
void bnp_tx(bnp_fragment_t * bnp_tx_p)
{
  if(bnp_tx_p->bnp_header.tx_number == DEFAULT_VALUE)
  {
    bnp_tx_p->bnp_header.tx_number = server_transaction_id; 
    server_transaction_id++;
    if((server_transaction_id & 0xFFFF) > 0x8ffff)
    {
      server_transaction_id = 1;
    }
  }
  
  if(bnp_tx_p->bnp_end.checksum == DEFAULT_VALUE)
  {
    bnp_tx_p->bnp_end.checksum = check_sum(bnp_tx_p);
  
  }
//  bnp_information.transaction_id = bnp_tx_p->bnp_header.tx_number;
//  if(bnp_information.transaction_id < 0x9000)
//  {
//    bnp_tx_p->bnp_data.u8[0] = INVALID_PARAM;
//    bnp_tx_p->bnp_header.length = 1;
//    bnp_tx_p->bnp_end.checksum = check_sum(&bnp_tx_frame);
//  }
  
  unsigned short bnp_len = 0;
  
  phy_fragment_t *ptr = (phy_fragment_t *)bnp_tx_p;
  //传输前需要进行包结构调整(将end_t贴在bcmp的payload之后)：保证字节流
  bnp_len = 
    bnp_tx_p->bnp_header.length 
      +sizeof(bnp_tx_p->bnp_end)
        +sizeof(bnp_tx_p->bnp_header);
  
  //需要验证算法是否正确
  memcpy(&(ptr->u8[bnp_len - sizeof(bnp_tx_p->bnp_end)]), (void *)&(bnp_tx_p->bnp_end), sizeof(bnp_tx_p->bnp_end));
  
  unsigned short custom_len = 0;//注意长度
  
  custom_bnp_send_data_t custom_send_data;
  custom_send_data.answer_delay         = 3000;
  custom_send_data.data_bnp_opcode      = bnp_tx_p->bnp_header.opcode;
  
  if(((bnp_tx_p->bnp_header.opcode & 0xF0) == BNP_RESPONSE_PACKAGE)//0xB0
     ||(bnp_tx_p->bnp_header.opcode == BNP_CLIENT_REQEUST_NOSUPPORT_ACK))//0xE1
  {
     custom_send_data.is_data_need_answer  = 0;
  }
  else//need response from device
    custom_send_data.is_data_need_answer   = 1;
  
  memcpy(custom_send_data.phy_valid_data.u8, ptr->u8, bnp_len);
  
  custom_len = 4 + bnp_len;
  
  if(bnp_tx_queue_ptr != NULL)
  {
    bool ret =false;
    ret = push_to_queue(bnp_tx_queue_ptr, (void *)&custom_send_data, custom_len);
    if(ret !=true)
    {
      log_warning("bnp_tx_queue_ptr full!");          
    }
  }
}

void bnp_client_heart_beat_req_func(bnp_fragment_t *bnp_p)           /*-0x1-BNP_CLIENT_HEART*/
{

  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_HEART_BEAT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   if(bnp_information.is_connected)
    bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = SUCCESS_NO_PROBLEM;
   else
    bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = PRO_UNCONNECTED;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);
   
   
}
void bnp_client_conn_req_func(bnp_fragment_t *bnp_p)		/*-0x2-BNP_CLIENT_CONN*/
{
  
  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_CONNECT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
 
   bnp_tx_frame.bnp_data.bnp_content_client_conn_reply.result = SUCCESS_NO_PROBLEM;
   
   if(bnp_information.is_connected == false)
     bnp_information.is_connected =true;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);

}


void bnp_send_disconn_req_func()
{
  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_C_S_DISCONNECT_REQUEST;
   
   bnp_tx_frame.bnp_header.tx_number = server_transaction_id;
   server_transaction_id++;
   if(server_transaction_id > 0x8fff)
   {
     server_transaction_id = 0;
   }
    
   bnp_tx_frame.bnp_header.length = 0;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);

}
void bnp_c_s_disconn_req_func(bnp_fragment_t *bnp_p)		/*-0x3-BNP_C_S_DISCONN*/
{
  
  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_C_S_DISCONNECT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
 
   bnp_tx_frame.bnp_data.bnp_content_client_disconn_reply.result = SUCCESS_NO_PROBLEM;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);
   
   bnp_information.is_connected = false;

}


void bnp_c_s_disconn_reply_func(bnp_fragment_t *bnp_p)		/*-0x3-BNP_C_S_DISCONN*/
{
   bnp_information.is_connected = false;

}

static void bnp_send_data_msg_ack(bnp_fragment_t *bnp_p)
{
   /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_DATA_MSG_ACK;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   bnp_tx_frame.bnp_header.length = 0;//data_msg_ack payload = 0
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);

}

//recv and analyse
void bnp_data_msg_func(bnp_fragment_t *bnp_p)		/*-0x4-BNP_DATA_MSG*/
{
   bnp_send_data_msg_ack(bnp_p);//repley ack
   
  if(bnp_information.is_connected)
  {  
   if(bcmp_analyse_callback_func != NULL)
   /*exec bcmp function*/
   bcmp_analyse_callback_func(bnp_p->bnp_data.bnp_content_data_msg);
  }
  else
  {
    /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_CONNECT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = PRO_UNCONNECTED;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);
  
  }

}


void bnp_get_data_msg_ack_func(bnp_fragment_t *bnp_p)		/*-0x4-BNP_DATA_MSG_ACK*/
{
  //send next bnp package
}


void bnp_nosupport_opcode_req_func(bnp_fragment_t *bnp_p)		/*-0xE1-BNP_NOSUPPORT*/
{
  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = BNP_HEADER_FLAG;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_REQEUST_NOSUPPORT_ACK;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   bnp_tx_frame.bnp_header.length = 0;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = BNP_END_FLAG;
   
   bnp_tx(&bnp_tx_frame);
  
}


/*
Define a function list, and register the corresponding function is used to 
deal with BNP instructions
*/
//类比结构体赋值
//在调用这个接口时，先判断tx_number是否正确
static const volatile BNP_process_list_t bnp_process_list[]=
{
  {NULL,                                NULL},					/*-0x0-BNP_INVALID*/
  {bnp_client_heart_beat_req_func,      NULL},      /*-0x1-BNP_CLIENT_HEART*/
  {bnp_client_conn_req_func,	        NULL},		/*-0x2-BNP_CLIENT_CONN*/
  {bnp_c_s_disconn_req_func,	        bnp_c_s_disconn_reply_func},		/*-0x3-BNP_C_S_DISCONN*/
  {bnp_data_msg_func,	                bnp_get_data_msg_ack_func},		        /*-0x4-BNP_DATA_MSG*/
 
};

extern volatile RingQueue_t usb_rx_queue_ptr;
void bnp_parse_task(void *p)
{
//  static unsigned int run_count = 0;
//
//  run_count++;

  //log_debug("[bnp_parse_task] is running: %d", run_count);

  unsigned char rx_bnp_buffer[250];
  int rx_len =0;
  
  unsigned short bnp_end_index =  0;
  unsigned short checksum_value =  0x0000;
  
  bool ret =false;
  if(usb_rx_queue_ptr !=NULL)
  {
     ret = take_from_queue(bnp_rx_queue_ptr, &rx_bnp_buffer, &rx_len, true);
     if(ret == true)
     {
        phy_fragment_t *ptr = (phy_fragment_t *)(rx_bnp_buffer);//类型转换
        
        //计算end标识的位置
        bnp_end_index = 
          sizeof(ptr->bnp_fragment.bnp_header)
            +ptr->bnp_fragment.bnp_header.length
              +sizeof(ptr->bnp_fragment.bnp_end.checksum);
        
        //计算checksum
        checksum_value = check_sum(&ptr->bnp_fragment);
        
        unsigned short rx_checksum =0;
        rx_checksum += ptr->u8[bnp_end_index -2] ;//低字节在前
        rx_checksum += (ptr->u8[bnp_end_index -1]<<8 & 0xFF00);//高字节在后
        
        //重新复制
        ptr->bnp_fragment.bnp_end.checksum = rx_checksum;
        ptr->bnp_fragment.bnp_end.end_flag = ptr->u8[bnp_end_index];
          
        if(
           (ptr->bnp_fragment.bnp_header.start_flag == BNP_HEADER_FLAG)   //0x7e
           && (ptr->u8[bnp_end_index] == BNP_END_FLAG)                    //0x3e
           && (checksum_value == rx_checksum)     //check bnp
           )
        {//bnp_okay
           log_debug("bnp rx okay, bnp_opcode: [0x%2x]", ptr->bnp_fragment.bnp_header.opcode);
           
           if(ptr->bnp_fragment.bnp_header.opcode > BNP_DATA_MSG_ACK)//超出范围
           {
             bnp_nosupport_opcode_req_func(&ptr->bnp_fragment);
             return;
           }
           
           if(bnp_process_list[ptr->bnp_fragment.bnp_header.opcode & 0x0F].bnp_rx_req !=NULL)
           {
              if((ptr->bnp_fragment.bnp_header.opcode & 0xF0) == BNP_RESPONSE_PACKAGE)//ack
              {
                bnp_rx_response_flag = true;//成功接收bnp response，关闭超时重发功能            
              }
              
              if(ptr->bnp_fragment.bnp_header.opcode == BNP_C_S_DISCONNECT_REPLY)//disconnect ack
              {
                bnp_c_s_disconn_reply_func(&ptr->bnp_fragment);
              }
              else if(ptr->bnp_fragment.bnp_header.opcode == BNP_DATA_MSG_ACK)//data ack
              {
                bnp_get_data_msg_ack_func(&ptr->bnp_fragment);
              }
              else//request
              {
                bnp_process_list[ptr->bnp_fragment.bnp_header.opcode & 0x0F].bnp_rx_req(&ptr->bnp_fragment); 
              }
           }
           
         }
        else
        {
          log_warning("bnp rx err! ");
          for(unsigned short i =0;i<rx_len;i++)
          {
            log_warning("data[%d]:[0x%2x]",i, ptr->u8[i]);
          }
//          log_warning("bnp rx err! sf:[0x%x], tx_numb:[0x%x], ef:[0x%x], rx_check:[0x%x], c_s:[0x%x]", 
//                    ptr->bnp_fragment.bnp_header.start_flag,
//                    ptr->bnp_fragment.bnp_header.tx_number,
//                    ptr->u8[bnp_end_index],
//                    rx_checksum,
//                    checksum_value);
        }
        
       }
     }       
  }

extern void set_timer_task(unsigned char       timer_id, 
                            unsigned int        delay, 
                            unsigned char       rearm, 
                            handler              timehandler, 
                            void *               param );

extern void phy_slip_init();

void bnp_init()
{

  bnp_rx_queue_ptr = create_queue(5, sizeof(bnp_fragment_t));
  if(bnp_rx_queue_ptr == NULL)
  {
    return;
  }
  
  
  bnp_information.is_connected = false;
  bnp_information.transaction_id = NULL;
  
  bnp_set_bcmp_analyse_callback(bcmp_parse_func);
  
  set_timer_task(BNP_PARSE_TASK, TIME_BASE_10MS*5, true, bnp_parse_task, NULL);
  
}

void protocol_init()
{
  
  phy_slip_init();
  
  bnp_init();
  
  bcmp_init();
  
  log_debug("protocol init has already been completed.");
  
}
