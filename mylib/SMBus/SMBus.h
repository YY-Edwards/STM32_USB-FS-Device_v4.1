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

#ifndef __SMBUS_H__
#define __SMBUS_H__

// Direct590 LTC4015 specific defines
// todo - need to handle the ePot.
#define CAT5140Register0       0x00                       // CAT5140ZI-xx-GT3 volatile register address
#define CAT5140Register8       0x08                       // CAT5140ZI-xx-GT3 volatility control register address
#define CAT5140Register1       0x01
#define SetCAT5140Permanent    0x00                       // CAT5140ZI-xx-GT3 'permanent' command
#define SetCAT5140Volatile     0x80                       // CAT5140ZI-xx-GT3 'volatile' command
#define CAT5140Addr            0x28
#define WriteCAT5140Addr       0x50                       // CAT5140ZI-xx-GT3 I2C write address
#define ReadCAT5140Addr        0x51                       // CAT5140ZI-xx-GT3 I2C read address

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "LTC4015.h" // needed for LTC4015_LION and port_configuration_t

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
extern LTC4015 chip;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int8_t SMBus_Init(void);

/* Function should return 0 on success and a non-0 error code on failure. The API functions will return your error code.
 */
int SMBus_Reg16_Write(uint8_t addr,                            //!< IC's Register address
                      uint8_t command_code,                    //!< SMBus command code for read.
                      uint16_t data,                           //!< Whole register data to be written to communication interface.
                      port_configuration_t *port_configuration //!< Any necessary communication interface configuration information, such as a file descriptor or control register location.
                      );
/*  Function should return 0 on success and a non-0 error code on failure. The API functions will return your error code.
 */
int SMBus_Reg16_Read(uint8_t addr,                             //!< IC's Register address
                     uint8_t command_code,                     //!< SMBus command code for write.
                     uint16_t *data,                           //!< Memory location to store whole register data read from communication interface.
                     port_configuration_t *port_configuration  //!< Any necessary communication interface configuration information, such as a file descriptor or control register location.
                     );

/*  Function should return 0 on success and a non-0 error code on failure. The API functions will return your error code.
 */
int SMBus_ARA_Read(uint8_t *addr,                            //!< Memory location for IC's Register address returned by ARA.
                   port_configuration_t *port_configuration  //!< Any necessary communication interface configuration information, such as a file descriptor or control register location.
                   );

void MakeCAT5140Permanent(void);
void MakeCAT5140Volatile(void);
uint8_t Get_CAT5140_ID(void);
void CAT5140_Optional_Handle();
void CAT5140_Write_Handle(uint8_t addr,  uint8_t command_code,  uint8_t data);
void CAT5140_Read_Handle(uint8_t addr,  uint8_t command_code,  uint8_t *data);

#endif

