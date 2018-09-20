/*
 Linear Technology DC2039A Demonstration Board.
 DC2039A Application File for Controlling the LTC4015 Nanopower Buck-Boost DC/DC with Integrated Coulomb Counter on the DC2039A PCB.

 @verbatim
 todo
 @endverbatim

 http://www.linear.com/product/LTC4015#demoboards

 REVISION HISTORY
 $Revision: 847 $
 $Date: 2014-11-06 15:21:01 -0500 (Thu, 06 Nov 2014) $

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

#ifndef __TYPEDEFS_H__
#define __TYPEDEFS_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <stdbool.h>
#include <stdint.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// PIC16 is little endian.
//#define BIG_ENDIAN

// define null if not already defined.
#ifndef NULL
#define NULL 0
#endif

// It bugs me for stdint.h types to end in "_t" but for the stdbool.h types to not.
#define bool_t bool

// macros for retrieving parts of variables in an endian safe way
#define UPPER_NIBBLE(x)     ((uint8_t)((x) >> 4) & 0x0f)
#define LOWER_NIBBLE(x)     ((uint8_t)(x) & 0x0f)
#define UPPER_BYTE(x)       ((uint8_t)(((x) >> 8) & 0xff))
#define LOWER_BYTE(x)       ((uint8_t)((x) & 0xff))
#define UPPER_WORD(x)       ((uint16_t)(((x) >> 16) & 0xffff))
#define LOWER_WORD(x)       ((uint16_t)((x) & 0xffff))

// retrieves the last byte from an arbitrarily sized array of bytes
#define LAST_BYTE(x)        ((((uint8_t*)(x)))[sizeof(x) - 1])
#define LAST_BYTE_ADDR(x)   (&LAST_BYTE(x))

// macros for common operations
#define UNUSED(x)           (void)(x)
#define FWD_DECLARE         extern
#define MIN(x, y)           (x < y ? x : y)
#define MAX(x, y)           (x > y ? x : y)
#define TOGGLE(x)           (x = (x ? 0 : 1))
#define MASK(size, shift)   (((1LL << (size)) - 1) << (shift))
#define BITVAL(value, bit)  ((value & MASK(1,bit)) >> bit)
#define STRINGIZE2(X)       #X
#define STRINGIZE(X)        STRINGIZE2(X)

// macros to return the number of bits needed to store a number
#define NBITS2(n)           ((n&2)?1:0)
#define NBITS4(n)           ((n&(0xC))?(2+NBITS2(n>>2)):(NBITS2(n)))
#define NBITS8(n)           ((n&0xF0)?(4+NBITS4(n>>4)):(NBITS4(n)))
#define NBITS16(n)          ((n&0xFF00)?(8+NBITS8(n>>8)):(NBITS8(n)))
#define NBITS32(n)          ((n&0xFFFF0000)?(16+NBITS16(n>>16)):(NBITS16(n)))
#define NBITS(n)            (n==0?0:NBITS32(1L*n)+1)

//! @name ENDIAN DEPENDENT BYTE INDEXES
//! @{
//! PIC16 is a Little Endian Device, where the least significant byte is stored in the first byte of larger data types.
#ifdef BIG_ENDIAN
#define LSB 1 //!< Location of Least Signficant Byte when Word is accessed as Byte Array
#define MSB 0 //!< Location of Most Signficant Byte when Word is accessed as Byte Array
#define LSW 1 //!< Location of Least Signficant Word when Long Word is accessed as Byte Array
#define MSW 0 //!< Location of most Signficant Word when Long Word is accessed as Byte Array
#else
#define LSB 0 //!< Location of Least Signficant Byte when Word is accessed as Byte Array
#define MSB 1 //!< Location of Most Signficant Byte when Word is accessed as Byte Array
#define LSW 0 //!< Location of Least Signficant Word when Long Word is accessed as Byte Array
#define MSW 1 //!< Location of most Signficant Word when Long Word is accessed as Byte Array
#endif
//! @}

// constants that improve code readability
#define BITS_PER_NIBBLE     4
#define BITS_PER_BYTE       8
#define UNICODE_PER_ASCII   2
#define MA_PER_A            1000
#define MS_PER_S            1000
#define US_PER_MS           1000
#define US_PER_S            1000000
#define MV_PER_V            1000
#define UV_PER_V            1000000
#define PERCENT_MAX         100

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
