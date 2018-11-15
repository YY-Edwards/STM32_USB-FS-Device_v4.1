
/* Includes ------------------------------------------------------------------*/
#include "BNP.h"
#include "BCMP.h"

static volatile bnp_information_t bnp_information;
static volatile unsigned short server_transaction_id = 1;
volatile RingQueue_t usb_rx_queue_ptr = NULL;

/*Defines the callback function is used to handle BCMP*/
void ( *bcmp_analyse_callback_func)(const bnp_content_data_msg_t) = NULL;

void bnp_set_bcmp_analyse_callback( void ( *func)(const bnp_content_data_msg_t))
{
  
  bcmp_analyse_callback_func = func;

}
  

static unsigned short check_sum (const bnp_fragment_t * bnp_p)
{
	
	unsigned short  sumScratch = 0;
        
        sumScratch = 
          bnp_p->bnp_header.opcode 
            + bnp_p->bnp_header.tx_number
              + bnp_p->bnp_header.length;
        
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
  
  
}

void bnp_client_heart_beat_req_func(bnp_fragment_t *bnp_p)           /*-0x1-BNP_CLIENT_HEART*/
{

  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = bnp_p->bnp_header.start_flag;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_HEART_BEAT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   if(bnp_information.is_connected)
    bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = SUCCESS_NO_PROBLEM;
   else
    bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = PRO_UNCONNECTED;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = bnp_p->bnp_end.end_flag;
   
   bnp_tx(&bnp_tx_frame);
   
   
}
void bnp_client_conn_req_func(bnp_fragment_t *bnp_p)		/*-0x2-BNP_CLIENT_CONN*/
{
  
  /*BNP frame will be send*/	
   bnp_fragment_t bnp_tx_frame;
  
   bnp_tx_frame.bnp_header.start_flag = bnp_p->bnp_header.start_flag;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_CONNECT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
 
   bnp_tx_frame.bnp_data.bnp_content_client_conn_reply.result = SUCCESS_NO_PROBLEM;
   
   if(bnp_information.is_connected == false)
     bnp_information.is_connected =true;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = bnp_p->bnp_end.end_flag;
   
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
  
   bnp_tx_frame.bnp_header.start_flag = bnp_p->bnp_header.start_flag;
   bnp_tx_frame.bnp_header.opcode = BNP_C_S_DISCONNECT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
 
   bnp_tx_frame.bnp_data.bnp_content_client_disconn_reply.result = SUCCESS_NO_PROBLEM;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = bnp_p->bnp_end.end_flag;
   
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
  
   bnp_tx_frame.bnp_header.start_flag = bnp_p->bnp_header.start_flag;
   bnp_tx_frame.bnp_header.opcode = BNP_DATA_MSG_ACK;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   bnp_tx_frame.bnp_header.length = 0;//data_msg_ack payload = 0
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = bnp_p->bnp_end.end_flag;
   
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
  
   bnp_tx_frame.bnp_header.start_flag = bnp_p->bnp_header.start_flag;
   bnp_tx_frame.bnp_header.opcode = BNP_C_S_DISCONNECT_REPLY;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   if(bnp_information.is_connected)
    bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = SUCCESS_NO_PROBLEM;
   else
    bnp_tx_frame.bnp_data.bnp_content_client_heart_reply.result = PRO_UNCONNECTED;
   
   bnp_tx_frame.bnp_header.length = 1;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = bnp_p->bnp_end.end_flag;
   
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
  
   bnp_tx_frame.bnp_header.start_flag = bnp_p->bnp_header.start_flag;
   bnp_tx_frame.bnp_header.opcode = BNP_CLIENT_REQEUST_NOSUPPORT_ACK;
   bnp_tx_frame.bnp_header.tx_number = bnp_p->bnp_header.tx_number;
   
   bnp_tx_frame.bnp_header.length = 0;
   
   bnp_tx_frame.bnp_end.checksum = check_sum(&bnp_tx_frame);
   
   bnp_tx_frame.bnp_end.end_flag = bnp_p->bnp_end.end_flag;
   
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



void bnp_init()
{
  
   if(usb_rx_queue_ptr!=NULL)
  {
   for (unsigned int i = 0; i < (usb_rx_queue_ptr->queue_deep); i++)
    {
      if(((usb_rx_queue_ptr->queue_point + i)->data)!= NULL)
      {
        free((usb_rx_queue_ptr->queue_point + i)->data);
        (usb_rx_queue_ptr->queue_point + i)->data = NULL;
      }
    }
     free(usb_rx_queue_ptr);
     usb_rx_queue_ptr =NULL;
  }
  
  usb_rx_queue_ptr = malloc(sizeof(ring_queue_t));
  if(usb_rx_queue_ptr ==NULL)
  {
    //printf("malloc ble_msg_queue_ptr failure\r\n");
    return ;
  }
  usb_rx_queue_ptr->head            = 0;
  usb_rx_queue_ptr->tail            = 0;
  usb_rx_queue_ptr->queue_deep      = 20;
  usb_rx_queue_ptr->data_size       = 64;
  usb_rx_queue_ptr->queue_point     = NULL;
  
  
  bool ret = init_queue(usb_rx_queue_ptr);
  
  if(ret == false)return;

  
  bnp_information.is_connected = false;
  bnp_information.transaction_id = NULL;
  
  bnp_set_bcmp_analyse_callback(bcmp_parse_func);
}

void bnp_parse_task(void *p)
{
   static unsigned int run_count = 0;
   run_count++;
   
   log_debug("[bnp_parse_task] is running: %d", run_count);

}

extern void set_timer_task(unsigned char       timer_id, 
                            unsigned int        delay, 
                            unsigned char       rearm, 
                            handler              timehandler, 
                            void *               param );
void protocol_init()
{
  bnp_init();
  
  bcmp_init();
  
  set_timer_task(BNP_PARSE_TASK, 5*TIME_BASE_500MS, true, bnp_parse_task, NULL);
  
  set_timer_task(BCMP_SEND_TASK, 6*TIME_BASE_500MS, true, bcmp_send_task, NULL);
  
  log_debug("protocol init has already been completed.");
  
}
