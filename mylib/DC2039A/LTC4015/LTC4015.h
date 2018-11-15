/*
LTC4015: Multichemistry Buck Battery Charger Controller with Digital Telemetry System

@verbatim
The  LTC®4015  is  a  complete  synchronous  buck  controller/charger  with pin
selectable,  chemistry specific charging and termination algorithms. The LTC4015
can  charge  Li-Ion/Polymer,  LiFePO4,  or  leadacid  batteries.  Battery charge
voltage  is  pin  selectable and I²C adjustable. Input current limit and charge
current   can   be  accurately  programmed  with  sense  resistors  and  can  be
individually  adjusted  via  the  I²C  serial  port. A digital telemetry system
monitors  all  system  power  parameters.  Safety  timer and current termination
algorithms  are  supported  for  lithium  chemistry  batteries. The LTC4015 also
includes  automatic  recharge, precharge (Li-Ion) and NTC thermistor protection.
The LTC4015's I²C port allows user customization of charger algorithms, reading
of  charger  status  information, configuration of the maskable and programmable
alerts,  plus  use  and  configuration  of  the  Coulomb counter. Available in a
38-Lead 5mm × 7mm QFN package.
@endverbatim

http://www.linear.com/product/LTC4015

http://www.linear.com/product/LTC4015#demoboards

REVISION HISTORY
$Revision: $
$Date: $

Copyright (c) 2016, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1.  Redistributions  of source code must retain the above copyright notice, this
    list  of conditions and the following disclaimer.

2.  Redistributions  in  binary  form must reproduce the above copyright notice,
    this  list of conditions and  the following disclaimer in the  documentation
    and/or other materials provided with the distribution.

THIS  SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY  EXPRESS  OR  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES   OF  MERCHANTABILITY  AND  FITNESS  FOR  A  PARTICULAR  PURPOSE  ARE
DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY  DIRECT,  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING,  BUT  NOT  LIMITED  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS  OF  USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY  THEORY  OF  LIABILITY,  WHETHER  IN  CONTRACT,  STRICT  LIABILITY,  OR TORT
(INCLUDING  NEGLIGENCE  OR  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The  views and conclusions contained in the software and documentation are those
of  the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However,  the Linduino is only possible because of the Arduino team's commitment
to   the   open-source   community.   Please,  visit  http://www.arduino.cc  and
http://store.arduino.cc  ,  and  consider  a  purchase that will help fund their
ongoing work.

Generated on: 2016-01-08
*/


/*! @file
 *  @ingroup LTC4015
 *  @brief LTC4015 communication library core header file defining
 *  prototypes, data structures and constants used by LTC4015.c
 *
 *  Functions  matching  the  prototypes  of  @ref  smbus_write_register and @ref
 *  smbus_read_register  must  be  provided  to this API. They will implement the
 *  SMBus  read  and write transactions on your hardware. If the register size of
 *  the  LTC4015  is 8 bits, functions should be provided that implement SMBus
 *  read  byte and write byte. If the register size of the LTC4015 is 16 bits,
 *  functions  should  be provided that implement SMBus read word and write word.
 *  smbus_read_register  needs  to  store the value read from the LTC4015 into
 *  the  variable  data.  Both  functions  should return 0 on success and a non-0
 *  error  code  on  failure.  The  API functions will return your error codes on
 *  failure and a 0 on success.
 */

#ifndef LTC4015_H_
#define LTC4015_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC4015_reg_defs.h"
#include "LTC4015_formats.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#pragma pack(1)  
// Type declarations
  /*! Hardware port information. Modify as needed for local hardware
  requirements. Available to user supplied read and write functions. */
  typedef struct
  {
    int file_descriptor; //!< Linux SMBus file handle
  } port_configuration_t;
  /*! Prototype of user supplied SMBus write_byte or write_word function. Should return 0 on success and a non-0 error code on failure. */
  typedef int (*smbus_write_register)(uint8_t addr, //!< Target IC's SMBus address
                                      uint8_t command_code, //!< Command code to be written to
                                      uint16_t data, //!< Data to be written
                                      port_configuration_t *port_configuration //!< Pointer to a user supplied port_configuration struct
                                     );
  /*! Prototype of user supplied SMBus read_byte or read_word function. Should return 0 on success and a non-0 error code on failure. */
  typedef int (*smbus_read_register)(uint8_t addr, //!< Target IC's SMBus address
                                     uint8_t command_code, //!< command code to be read from
                                     uint16_t *data, //!< Pointer to data destination
                                     port_configuration_t *port_configuration //!< Pointer to a user supplied port_configuration struct
                                    );
  typedef void *LTC4015;
  /*! Information required to access hardware SMBus port */
  typedef struct
  {
    uint8_t addr; //!< Target IC's SMBus address
    smbus_read_register read_register; //!< Pointer to a user supplied smbus_read_register function
    smbus_write_register write_register; //!< Pointer to a user supplied smbus_write_register function
    port_configuration_t *port_configuration; //!< Pointer to a user supplied port_configuration struct
  } LTC4015_chip_cfg_t;
  
   typedef struct
  { 
    unsigned short intvcc_gt_2p8v            :1;//0
    unsigned short intvcc_gt_4p3v            :1;//1
    unsigned short vin_gt_vbat               :1;//2
    unsigned short vin_ovlo                  :1;//3
    unsigned short thermal_shutdown          :1;//4
    unsigned short no_rt                     :1;//5
    unsigned short ok_to_charge              :1;//6
    unsigned short reserved_1                :1;//7
    unsigned short cell_count_error          :1;//8
    unsigned short drvcc_good                :1;//9
    unsigned short equalize_req              :1;//10
    unsigned short mppt_en_pin               :1;//11
    unsigned short reserved_2                :1;//12
    unsigned short charger_enabled           :1;//13
    unsigned short reserved_3                :2;
  } LTC4015_system_status_t;

  typedef struct
  {
    unsigned short constant_voltage              :1;//0，恒压充电
    unsigned short constant_current              :1;//1，恒流充电
    unsigned short iin_limit_active              :1;//2，控制输入电流 
    unsigned short vin_uvcl_active               :1;//3，控制输入电压
    unsigned short reserved                      :12;
  } LTC4015_charge_status_t;
  
  typedef struct
  {
    unsigned short bat_short_fault              :1;//0, 充电器电池短路
    unsigned short bat_missing_fault            :1;//1，充电器没有接电池
    unsigned short max_charge_time_fault        :1;//2，充电器超过设定的最大充电时间
    unsigned short c_over_x_term                :1;//3，充电器库伦值满标志
    unsigned short timer_term                   :1;//4，充电器超过设定的最大恒压充电时间
    unsigned short ntc_pause                    :1;//5，充电器电池温度过高故障状态
    unsigned short cc_cv_charge                 :1;//6，充电器恒流恒压充电状态
    unsigned short precharge                    :1;//7，充电器预充状态
    unsigned short charger_suspended            :1;//8，充电器挂起状态
    unsigned short absorb_charge                :1;//9
    unsigned short equalize_charge              :1;//10
    unsigned short reserved                     :5;
    
  } LTC4015_charger_state_t;
  
  
//  typedef struct
//  {
//    signed short              VIN;//输入电压，单位mv
//    signed short              IIN;//输入电流，单位ma
//    signed short              VBAT;//单节电池电压，单位mv
//    signed short              IBAT;//充电电流，单位ma
//    signed short              VSYS;//负载端电压，单位mv
//    signed short              ISYS;//负载输出电流（评估值），单位ma
////    unsigned char               chemistry_type;//电池化学成分：参考如下所示
////                                /*   
////                                0x0:Li-lon Programmable
////                                0x1:Li-lon Fixed 4.2V/cell
////                                0x2:Li-lon Fixed 4.1V/cell
////                                0x3:Li-lon Fixed 4.0V/cell
////                                0x4:LiFePO4 Programmable
////                                0x5:LiFePO4 Fixed Fast Charge
////                                0x6:LiFePO4 Fixed 3.6V/cell
////                                0x7:Lead-Acid Fixed
////                                0x8:Lead-Acid Programmable
////                              */
//// 
////    unsigned char               cell_count;//电池节数
//    unsigned int                bat_total_capcity;
//    unsigned int                bat_currently_capacity;
//    //unsigned char               qcount_percent;//库伦百分比数值（0~100）
//    signed short                NTC;//电池温度，单位℃
//    signed short                DIE;//模具(主控芯片)温度，单位℃
//    unsigned char               battery_state;
//    unsigned char               alert_identifier;
//    
////    LTC4015_charge_status_t     charge_status;//充电状态
////    LTC4015_charger_state_t     charger_state;//实时的充电器形态（彼此独立）
//    //LTC4015_system_status_t     system_status;//系统状态
//    
//  } LTC4015_detail_info_t;
  
  
#pragma pack()
  // function declarations
  /*! Returns a pointer to a LTC4015 structure used by LTC4015_write_register and LTC4015_read_register */
  LTC4015 LTC4015_init(LTC4015_chip_cfg_t *cfg //!< Information required to access hardware SMBus port
                      );
  /*! Function to modify a bit field within a register while preserving the unaddressed bit fields */
  int LTC4015_write_register(LTC4015 chip_handle, //!< Struct returned by LTC4015_init
                             uint16_t registerinfo, //!< Bit field name from LTC4015_regdefs.h
                             uint16_t data //!< Data to be written
                            );
  /*! Retrieves a bit field data into *data. Right shifts the addressed portion down to the 0 position */
  int LTC4015_read_register(LTC4015 chip_handle, //!< Struct returned by LTC4015_init
                            uint16_t registerinfo, //!< Register name from LTC4015_regdefs.h
                            uint16_t *data //!< Pointer to the data destination
                           );
  /*! Multiple LTC4015 use.
    Multiple LTC4015s can be used with this API. Each one must be initialized.
    The LTC4015_init requires some memory for each LTC4015. This memory can
    be  statically  allocated  by  defining  MAX_NUM_LTC4015_INSTANCES  to the
    number of LTC4015s required. Alternatively it can be dynamically allocated
    by  defining  LTC4015_USE_MALLOC.  The  default  is  to not use malloc and
    statically allocate one LTC4015.
  */
    


#ifndef MAX_NUM_LTC4015_INSTANCES
#define MAX_NUM_LTC4015_INSTANCES 1
#endif
#ifdef __cplusplus
}
#endif
#endif /* LTC4015_H_ */
