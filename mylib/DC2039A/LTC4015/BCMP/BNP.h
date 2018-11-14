#ifndef BNP_H_
#define BNP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC4015.h"
#include "logger.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
  
#define BNP_HEADER_FLAG             (char)0x7E    
#define BNP_END_FLAG                (char)0x3E
#define DEFAULT_VALUE	            0x0000  
  
   
  
    typedef enum
  {
    SUCCESS_NO_PROBLEM  =0x00,
    FAILURE             =0x01,
    PRO_UNCONNECTED         =0x02,
    INVALID_PARAM       =0x03,
    UNSUPPORT_OPCODE    =0x04,
          
  }bnp_result_t;
  
  
     typedef struct
  {
    unsigned char       start_flag;        
    unsigned char       opcode;          
    unsigned short      tx_number;  //server:0x0001~0x8fff. client:0x9000~0xffff
    unsigned short      length;//length of bnp_data.
  } bnp_header_t;
  
  
    typedef enum
  {
    BNP_CLIENT_HEART_BEAT_REQUEST               = 0X01,
    BNP_CLIENT_HEART_BEAT_REPLY                 = 0XB1,
    BNP_CLIENT_CONNECT_REQUEST                  = 0X02,
    BNP_CLIENT_CONNECT_REPLY                    = 0XB2,
    BNP_C_S_DISCONNECT_REQUEST                  = 0X03,
    BNP_C_S_DISCONNECT_REPLY                    = 0XB3,
    BNP_DATA_MSG_REQUEST                        = 0X04,
    BNP_DATA_MSG_ACK                            = 0XB4,
    BNP_CLIENT_REQEUST_NOSUPPORT_ACK            = 0XE1,
          
  }bnp_opcode_enum;
  
  //heart beat
   typedef struct
  {
    unsigned char       unused[10]; 
  } bnp_content_client_heart_req_t;
  
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_client_heart_reply_t;
  
    //connect
   typedef struct
  {
    unsigned char       unused[10]; 
  } bnp_content_client_conn_req_t;
  
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_client_conn_reply_t;
  
   //disconnect
   typedef struct
  {
    unsigned char       unused[10]; 
  } bnp_content_client_disconn_req_t;
  
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_client_disconn_reply_t;
  
  
     //nosupport
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_nosupport_ack_t;
  
    
  
  
     //DATA
   typedef struct
  {
     unsigned short   bcmp_opcode;//request/reply/broadcast
     unsigned char    u8[120];
  } bnp_content_data_msg_t;
  
   typedef struct
  {
    unsigned char    u8[10];//This message contains no payload.
  } bnp_content_data_msg_ack_t;
  
  

  
/*DATA*/
typedef union {

        bnp_content_client_heart_req_t		                bnp_content_client_heart_req;
	bnp_content_client_heart_reply_t		        bnp_content_client_heart_reply;	
	bnp_content_client_conn_req_t		                bnp_content_client_conn_req;
	bnp_content_client_conn_reply_t		                bnp_content_client_conn_reply;	
        bnp_content_client_disconn_req_t		        bnp_content_client_disconn_req;//server<->client
	bnp_content_client_disconn_reply_t		        bnp_content_client_disconn_reply;	
	bnp_content_data_msg_t					bnp_content_data_msg;
	bnp_content_data_msg_ack_t				bnp_content_data_msg_ack;
        bnp_content_nosupport_ack_t				bnp_content_nosupport_ack;
        unsigned char                                           u8[122];
        
} bnp_data_t;
  
  
  
     typedef struct
  {
    unsigned short       checksum;//sum of bytes in BNP_fragment_t without header and end   
    unsigned char        end_flag;
  } bnp_end_t;
  
 
  typedef struct
  {
    bnp_header_t     bnp_header;        // 6  bytes
    bnp_data_t       bnp_data;          // n>=2  bytes
    bnp_end_t        bnp_end;           // 3  bytes
  } bnp_fragment_t;
    
  
  
  typedef struct
{	
    /*BNP status*/
    volatile bool is_connected;	
            
    /*Transaction ID*/
    volatile unsigned short transaction_id;
    
}bnp_information_t;
  

  typedef struct
{
	/*request function*/
	void (* bnp_rx_req)(bnp_fragment_t *);
	
	/*reply function*/
	void (* bnp_rx_reply)(bnp_fragment_t *);
	
}BNP_process_list_t;
  

#define MIN_RESEND_TIMES	3
  
void bnp_client_heart_beat_req_func(bnp_fragment_t *);           /*-0x1-BNP_CLIENT_HEART*/
void bnp_client_conn_req_func(bnp_fragment_t *);		/*-0x2-BNP_CLIENT_CONN*/
void bnp_c_s_disconn_req_func(bnp_fragment_t *);		/*-0x3-BNP_C_S_DISCONN*/
void bnp_c_s_disconn_reply_func(bnp_fragment_t *);		/*-0x3-BNP_C_S_DISCONN*/
void bnp_data_msg_func(bnp_fragment_t *);	                /*-0x4-BNP_DATA_MSG*/
void bnp_data_msg_ack_func(bnp_fragment_t *);	                /*-0x4-BNP_DATA_MSG*/

void bnp_nosupport_opcode_req_func(bnp_fragment_t *);		/*-0xE1-BNP_NOSUPPORT*/

void bnp_send_disconn_req_func();
  
/*register the bcmp analyse function(callback function) in bnp*/
void bnp_set_bcmp_analyse_callback( void ( *func)(const bnp_content_data_msg_t));
 
void bnp_init();
void bnp_tx(bnp_fragment_t * bnp_tx_p);

void protocol_init();
  

#ifdef __cplusplus
}
#endif
#endif /* BNP_H_ */











