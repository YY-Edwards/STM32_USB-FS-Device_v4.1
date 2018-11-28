/*
 Linear Technology DC2039A Demonstration Board.

 @verbatim
 This code implements a simple LT_printf function over the Arduino serial port.
 @endverbatim

 REVISION HISTORY
 $Revision: 4639 $
 $Date: 2016-01-29 16:42:40 -0500 (Fri, 29 Jan 2016) $

 Copyright (c) 2014, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of Linear Technology Corp.

 */

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "myiic.h"
#include "SMBus.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define SMBUS_ARA_ADDR      0x19

LTC4015_chip_cfg_t cfg =
{
  LTC4015_ADDR_68,          //.addr (7-bit)
  SMBus_Reg16_Read,         //.read_register
  SMBus_Reg16_Write,        //.write_register
  0                         //.port_configuration not used
};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
LTC4015 chip;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

int8_t SMBus_Init(void)
{
  int8_t result = 0;
  IIC_Init();//MCU iic 接口初始化
  chip = LTC4015_init(&cfg);   //! Configure read/write functions

  return result;
}

/*  Function should return  0  on  success and a non-0 error code on failure. The API functions will return your error code.
 */
int SMBus_Reg16_Read(uint8_t addr,                             //!< IC's Register address
                     uint8_t command_code,                     //!< SMBus command code for read.
                     uint16_t *data,                           //!< Memory location to store whole register data read from communication interface.
                     port_configuration_t *port_configuration  //!< Any necessary communication interface configuration information, such as a file descriptor or control register location.
                    )
{
  UNUSED(port_configuration);
  uint8_t temp_buffer[2];
  int8_t result = true;

  temp_buffer[0] = addr << 1;           // Write the command code address first.
  temp_buffer[1] = command_code;
  IIC_Start(LTC4015_IIC);
  if (result) result &= IIC_Write_Nbytes(LTC4015_IIC, temp_buffer, sizeof(temp_buffer));
  IIC_Start(LTC4015_IIC);
  temp_buffer[0] |= 1;                  // Now read the data for that command code.
  if (result) result &= IIC_Write_Nbytes(LTC4015_IIC, temp_buffer, sizeof(*temp_buffer));
  if (result) result &= IIC_Read_Nbytes(LTC4015_IIC, temp_buffer, sizeof(temp_buffer));
  IIC_Stop(LTC4015_IIC);
  if (result)
  {
    ((uint8_t*)data)[LSB] = temp_buffer[0];
    ((uint8_t*)data)[MSB] = temp_buffer[1];
  }

  return !result; // LTC driver expects 0 = good, where i2c driver returns True for success.
}

/* Function should return 0 on success and a non-0 error code on failure. The API functions will return your error code.
 */
int SMBus_Reg16_Write(uint8_t addr,                            //!< IC's Register address 
                      uint8_t command_code,                    //!< SMBus command code for write.
                      uint16_t data,                           //!< Whole register data to be written to communication interface.
                      port_configuration_t *port_configuration //!< Any necessary communication interface configuration information, such as a file descriptor or control register location.
                     )
{
  UNUSED(port_configuration);
  uint8_t temp_buffer[4];
  int8_t result =true;

  temp_buffer[0] = addr << 1;           // Write the command code address first and data for that command code.
  temp_buffer[1] = command_code;
  temp_buffer[2] = ((uint8_t*)&data)[LSB];
  temp_buffer[3] = ((uint8_t*)&data)[MSB];
  IIC_Start(LTC4015_IIC);
  if (result) result &= IIC_Write_Nbytes(LTC4015_IIC, temp_buffer, sizeof(temp_buffer));
  IIC_Stop(LTC4015_IIC);

  return !result; // LTC driver expects 0 = good, where i2c driver returns True for success.
}

/* Function should return 0 on success and a non-0 error code on failure. The API functions will return your error code.
 */
int SMBus_ARA_Read(uint8_t *addr,                           //!< Memory location for IC's Register address returned by ARA.
                   port_configuration_t *port_configuration //!< Any necessary communication interface configuration information, such as a file descriptor or control register location.
                  )
{
  UNUSED(port_configuration);
  uint8_t temp_buffer[1];
  int8_t result =true;

  // Read from the ARA address and look for the LTC address to be returned.
  temp_buffer[0] = SMBUS_ARA_ADDR;
  IIC_Start(LTC4015_IIC);
  if (result) result &= IIC_Write_Nbytes(LTC4015_IIC, temp_buffer, sizeof(*temp_buffer));
  if (result) result &= IIC_Read_Nbytes(LTC4015_IIC, temp_buffer, sizeof(temp_buffer));
  IIC_Stop(LTC4015_IIC);
  if (result) *addr = temp_buffer[0] >> 1;
  
  return !result; // LTC driver expects 0 = good, where i2c driver returns True for success.
}


void CAT5140_Read_Handle(uint8_t addr,  uint8_t command_code,  uint8_t *data)
{
   uint8_t temp_buffer[2];
  int8_t result = true;

  temp_buffer[0] = addr << 1;           // Write the command code address first.
  temp_buffer[1] = command_code;
  IIC_Start(CAT5140_IIC);
  if (result) result &= IIC_Write_Nbytes(CAT5140_IIC, temp_buffer, sizeof(temp_buffer));
  IIC_Start(CAT5140_IIC);
  temp_buffer[0] |= 1;                  // Now read the data for that command code.
  if (result) result &= IIC_Write_Nbytes(CAT5140_IIC, temp_buffer, sizeof(*temp_buffer));
  if (result) result &= IIC_Read_Nbytes(CAT5140_IIC, temp_buffer, 1);
  IIC_Stop(CAT5140_IIC);
  if (result)
  {
    *data = temp_buffer[0];
  }

}
                     
void CAT5140_Write_Handle(uint8_t addr,  uint8_t command_code,  uint8_t data)
{
  uint8_t temp_buffer[3];
  int8_t result =true;

  temp_buffer[0] = addr << 1;           // Write the command code address first and data for that command code.
  temp_buffer[1] = command_code;
  temp_buffer[2] = data;
  IIC_Start(CAT5140_IIC);
  if (result) result &= IIC_Write_Nbytes(CAT5140_IIC, temp_buffer, sizeof(temp_buffer));
  IIC_Stop(LTC4015_IIC);

}

void CAT5140_Optional_Handle()
{
  uint8_t r_value = 0;
  uint8_t w_value = 0;
  
  MakeCAT5140Volatile();
  CAT5140_Read_Handle(CAT5140Addr, CAT5140Register0, &r_value);
  w_value = 0x19;
  CAT5140_Write_Handle(CAT5140Addr, CAT5140Register0, w_value);
  r_value = 0;
  CAT5140_Read_Handle(CAT5140Addr, CAT5140Register0, &r_value);
  MakeCAT5140Permanent();
//  w_value = 0x7A;
//  CAT5140_Write_Handle(CAT5140Addr, CAT5140Register0, w_value);
//  
//  CAT5140_Read_Handle(CAT5140Addr, CAT5140Register0, &r_value);
  //MakeCAT5140Volatile();

}
uint8_t Get_CAT5140_ID(void)
{

  uint8_t dev_id =0;
  
  CAT5140_Read_Handle(CAT5140Addr, CAT5140Register1, &dev_id);
  
  return dev_id;

}

void MakeCAT5140Permanent(void)
{
  CAT5140_Write_Handle(CAT5140Addr, CAT5140Register8, SetCAT5140Permanent);
}
void MakeCAT5140Volatile(void)
{
  CAT5140_Write_Handle(CAT5140Addr, CAT5140Register8, SetCAT5140Volatile);
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
