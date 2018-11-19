#ifndef PHY_SLIP_H_
#define PHY_SLIP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "BNP.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "logger.h"
#include "task_timer.h"  



#define END             (char)0xC0    /* indicates end of packet */
#define ESC             (char)0xC1    /* indicates byte stuffing */
#define ESC_END         (char)0xC2   /* ESC ESC_END means END data byte */
#define ESC_ESC         (char)0xC3   /* ESC ESC_ESC means ESC data byte */ 
  
#pragma pack(1)  
  
typedef enum
{
  SLIP_FIND_HEADER,
  SLIP_READ_DATA,
  SLIP_WAIT_END,
}
SLIP_parser_state_enum;

    
typedef union {
  
    bnp_fragment_t      bnp_fragment;
    unsigned char       u8[150];
   
} phy_fragment_t;
    

typedef enum
{
  CUSTOM_ASSEMBLE_DATA,
  CUSTOM_WAIT_FOR_TX,
  CUSTOM_WAIT_RESPONSE,
}
custom_data_send_state_enum;
  
  typedef struct {
    
    unsigned char       data_bnp_opcode;//数据bnp操作码  
    unsigned char       is_data_need_answer;//是否需要应答
    unsigned short      answer_delay;//应答最大等待延迟，单位ms

    phy_fragment_t      phy_valid_data;
  
} custom_bnp_send_data_t;
    
  
//   typedef struct
//  {
//    unsigned char       slip_header;        
//    unsigned char       slip_translated_payload[150];
//    unsigned char       slip_end;
//  } phy_slip_fragment_t;
  
  
  
#pragma pack()
  
void phy_slip_init();
void Handle_USBAsynchXfer (void);
  
 
#ifdef __cplusplus
}
#endif
#endif /* PHY_SLIP_H_ */