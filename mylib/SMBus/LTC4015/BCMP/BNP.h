#ifndef BNP_H_
#define BNP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC4015.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
  
#define BNP_HEADER_FLAG             (char)0x7E    
#define BNP_END_FLAG                (char)0x3E
  
  
    typedef enum
  {
    SUCCESS,
    FAILURE,
    UNCONNECTED,
    INVALID_PARAM,
    UNSUPPORT_OPCODE,
          
  }bnp_result_t;
  
  
     typedef struct
  {
    unsigned char       start_flag;        
    unsigned char       opcode;          
    unsigned short      tx_number;  //server:0x0000~0x8fff. client:0x9000~0xffff
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
    BNP_CLIENT_NOSUPPORT_ACK                    = 0XE1,
          
  }bnp_opcode_enum;
  
  //heart beat
   typedef struct
  {
    unsigned char       unused[10]; 
  } bnp_content_device_heart_req_t;
  
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_device_heart_reply_t;
  
    //connect
   typedef struct
  {
    unsigned char       unused[10]; 
  } bnp_content_device_conn_req_t;
  
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_device_conn_reply_t;
  
   //disconnect
   typedef struct
  {
    unsigned char       unused[10]; 
  } bnp_content_device_disconn_req_t;
  
   typedef struct
  {
    unsigned char       result; 
    unsigned char       unused[9]; 
  } bnp_content_device_disconn_reply_t;
  
  
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

        bnp_content_device_heart_req_t		                bnp_content_device_heart_req;
	bnp_content_device_heart_reply_t		        bnp_content_device_heart_reply;	
	bnp_content_device_conn_req_t		                bnp_content_device_conn_req;
	bnp_content_device_conn_reply_t		                bnp_content_device_conn_reply;	
        bnp_content_device_disconn_req_t		        bnp_content_device_disconn_req;//server<->client
	bnp_content_device_disconn_reply_t		        bnp_content_device_disconn_reply;	
	bnp_content_data_msg_t					bnp_content_data_msg;
	bnp_content_data_msg_ack_t				bnp_content_data_msg_ack;
        bnp_content_nosupport_ack_t				bnp_content_nosupport_ack;
        
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
    
  
  
  
  
#endif
#ifdef __cplusplus
}
#endif
#endif /* BNP_H_ */








