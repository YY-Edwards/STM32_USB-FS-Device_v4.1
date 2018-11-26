#ifndef BCMP_H_
#define BCMP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "BNP.h"
#include "LTC4015.h"  
#include <stdint.h>
#include <stddef.h>
#include <string.h> 
  
  
  
  typedef bnp_content_data_msg_t  bcmp_fragment_t;//定义别名
  typedef bnp_result_t            bcmp_result_t;      
  
  
  #define BCMP_REQUEST		0X0000
  #define BCMP_REPLY	        0x8000
  #define BCMP_BRDCAST		0xB000
  
 
#pragma pack(1)
  
  /*
Battery Information: 0x0001.

Request:0x0001;
Query the information of battery;	
Payload is empty
Rely:0x8001:
Reply the battery information to client
Payload
Boardcast:0xB001:
Broadcast the battery information to client in cycle (3s), server actively send the broadcast when an Alert occurs for once.
Payload is Battery information
  
  */
  
  #define BATTERY_INFO 0x001
  
  
    typedef struct
  {
    unsigned char unused[3];
    
  } bcmp_battery_info_req_t;
  
  
    typedef struct
  {
    signed short              VIN;//输入电压，单位v，1000：1
    signed short              IIN;//输入电流，单位a，1000：1
    signed short              VCHARGER;//电池充电电压，单位v，1000：1
    signed short              ICHARGER;//电池充电电流，单位a，1000：1
    signed short              VSYS;//负载端电压，单位v，1000：1
    signed short              ISYS;//负载输出电流（评估值），单位a，1000：1

    unsigned int                bat_total_capacity;//，1000：1
    unsigned int                bat_currently_capacity;//，1000：1
    signed short                NTC;//电池温度，单位℃，100：1
    signed short                DIE;//模具(主控芯片)温度，单位℃，100：1
    unsigned char               battery_state;
    unsigned char               alert_identifier;
    unsigned int              discharge_time;//minute,预估放电时间，1：1
    unsigned int              remained_charge_time;//minute,充满剩余时间，1：1
    
  } bcmp_battery_info_brdcast_t;
  
  typedef struct
  {
    unsigned char                       result;
    bcmp_battery_info_brdcast_t          detailed_info;        
  
  } bcmp_battery_info_reply_t;
   
  
  
  /*
Battery Alert Configuration and Notify Service: 0x0002. 
This message is used to request notices of certain services of the server.
When a certain service is triggered, the server will send a notice message to 
the client which requested notice of such service. 
The service is not requested notice of by default when the server is powered on. 
Process of notice requesting and cancelling is shown in following Figure Process of notice requesting and cancelling. 
  
  */
  
    #define BATTERY_ALERT_CONFIG_NOTICE 0x002
  
      typedef enum
  {
    VBAT_LO_ALERT_LIMIT                         = 0XA001,
    VBAT_HI_ALERT_LIMIT                         = 0XA002,
    VIN_LO_ALERT_LIMIT                          = 0XA003,
    VIN_HI_ALERT_LIMIT                          = 0XA004,
    VSYS_LO_ALERT_LIMIT                         = 0XA005,
    VSYS_HI_ALERT_LIMIT                         = 0XA006,
    IIN_HI_ALERT_LIMIT                          = 0XA007,
    IBAT_LO_ALERT_LIMIT                         = 0XA008,
    DIE_TEMP_HI_ALERT_LIMIT                     = 0XA009,
    BAT_TEMP_HI_ALERT_LIMIT                     = 0XA00C,
    
    
    
    CONSTANT_VOLTAGE_ALERT                      = 0xB000,
    CONSTANT_CURRENT_ALERT	                = 0xB001,
    IIN_LIMIT_ACTIVE_ALERT	                = 0xB002,
    VIN_UVCL_ACTIVE_ALERT	                = 0xB003,
      
    BAT_SHORT_FAULT_ALERT	                = 0xC000,
    BAT_MISSING_FAULT_ALERT	                = 0xC001,
    MAX_CHARGE_TIME_FAULT_ALERT                 = 0xC002,
    MAX_CV_TIME_ALERT                           = 0xC004,
    BAT_NTC_PAUSE_ALERT                         = 0xC005,
    CC_CV_CHARGE_ALERT                          = 0xC006,
    PRECHARGE_ALERT                             = 0xC007,
    CHARGER_SUSPENDED_ALER                      = 0xC008,
    ABSORT_CHARGE_ALERT                         = 0xC009,
    EQUALIZE_CHARGE_ALERT                       = 0xC00A,
      
      
  }bcmp_alert_id_enum;
  
  
  
  
  typedef struct
  {
    unsigned char type;
    unsigned char operation;
    unsigned short id;
    unsigned short value;
  
  } bcmp_alert_notice_config_info_t;
  
    typedef struct
  {
    unsigned char number;
    bcmp_alert_notice_config_info_t config_array[15];//15>=number
  
  } bcmp_alert_notice_config_req_t;
  
  
 
  
  
  
  /*
Alert Information: 0x0003.
Request:0x0003;
Query the alert information of charger;	
Payload is empty.
Rely:0x8003:
Reply the alert information to client;
Payload: 
Opcode [1] + Result [2] + Number [1] + {Type [1] + Id [2] + Value [2]} [Number]

  */
  
  
   #define ALERT_INFO 0x003
  
   #define ALERT_INFO_RESULT_NOTHING 0x00
   #define ALERT_INFO_RESULT_HAPPEND 0x01
   #define MAX_ALERT_COUNT              15
  
  typedef enum
  {
    CHAERGER_STATE_ALERT                         = 0X00,
    CHAERGE_STATUS_ALERT                         = 0X01,
    LIMIT_ALERT                                  = 0X02,
     
  }bcmp_alert_type_enum;

  typedef enum
  {
    BAT_CHARGING                                     = 0X00,
    BAT_DISCHARGING                                  = 0X01,
    BAT_IDLE_SUSPEND                                 = 0X02,
    BAT_ABNORMAL                                     = 0X03,
    
  }bcmp_battery_state_enum;
  
  
     typedef struct
  {
    unsigned char            type;//bcmp_alert_type_enum
    unsigned short            id;//bcmp_alert_id_enum
    
    /*
    
    If type is equal to 0x00 or 0x01, the value is invalid. 
    If type is equal to 0x02, the value is an alert limit (signed).
 
    */
    signed short              value;
    
  } bcmp_alert_detailed_info_t;
  
    typedef struct
  {
    unsigned char                       unused;
  } bcmp_alert_info_req_t;
  
  typedef struct
  {
    unsigned char                       result;
    unsigned char                       number;
    bcmp_alert_detailed_info_t           alert_detailed_info[MAX_ALERT_COUNT];  //15>=number      
  
  } bcmp_alert_info_reply_t;
  
  
  
  
    /*
Configure Charger settings: 0x0004.
Request:0x0004;
Request to configure the charger setting;	
Opcode [2] + UVCL [2] + JEITA [1] + VCHARGECELL [2] + ICHARGECELL [2] + IINLIM[2] + MCVTIME[2] 
  + MCHARGETIME[2] + BATCAPACITY[4] + CHARGESTATUSALERTS[2] + CHARGERSTATEALERTS[2]
  + {Id [2] + Operation [1] + Value [2]} [10]
  
Rely:0x8004:
Reply the request of configuration;
Payload is empty


  */
  
     #define CONFIG_CHARGER_SETTINGS    0x004
     #define LIMIT_CONFIG_COUNT         10
  
    typedef struct
  {
    unsigned short                      id;
    unsigned char                       operation;
    unsigned short                      value;
  } bcmp_limit_config_t;
  
  
      typedef struct
  {
    unsigned short                      uvcl;
    unsigned char                       jeita;
    unsigned short                      vcharge_cell;
    unsigned short                      icharge_cell;
    unsigned short                      iinlim;
    unsigned short                      max_cv_time;
    unsigned short                      max_charge_time;
    unsigned int                        bat_capacity;
    LTC4015_charge_status_t              charge_status;
    LTC4015_charger_state_t              charger_state;
    bcmp_limit_config_t                  limits[LIMIT_CONFIG_COUNT];
    
  } bcmp_config_req_t;
  

  
  typedef bcmp_config_req_t charger_settings_t;
  
typedef struct
{
  /*request function*/
  void (* bcmp_rx_req_and_response_func)(const bcmp_fragment_t *);
  
//  /*reply function*/
//  void (* bcmp_rx_reply)(void *);
//  
//  /*board cast function*/
//  void (* bcmp_rx_brdcst)(void *);
  
}BCMP_process_list_t;  
  
#pragma pack()

  
void bcmp_init(void);

void bcmp_opcode_not_support_reply(void);
void bcmp_parse_func(const bcmp_fragment_t);
//void bcmp_send_task(void *p);
  
    

#ifdef __cplusplus
}
#endif
#endif /* BCMP_H_ */
