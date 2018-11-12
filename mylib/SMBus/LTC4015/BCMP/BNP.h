#ifndef BNP_H_
#define BNP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC4015.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
  
 
  
  
  typedef struct
  {
    bnp_header_t     bnp_header;  // 2  Words
    bnp_data_t       bnp_data;  // 2  Words
    bnp_end_t        bnp_end;  // 2  Words
  } BNP_fragment_t;
    
  
  
  //Fixed size 256 byte Rx fragment.
typedef struct {
	phy_header_t     phy_header;  // 2  Words
	xnl_header_t     xnl_header;  // 6  Words
	xnl_payload_t    xnl_payload; // 7  ,10hWords (most common messages)
	U16            rest[19];
} xnl_fragment_t;
  
  
#endif
#ifdef __cplusplus
}
#endif
#endif /* BNP_H_ */








