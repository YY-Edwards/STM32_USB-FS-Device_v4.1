#ifndef DC2039A_H_
#define DC2039A_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "LTC4015.h"
#include "SMBus.h"
#include "logger.h"
#include "sys.h"  
#include "delay.h"  
#include <stdint.h>
#include <stddef.h>
#include <string.h>
  
  
  
  
void DC2039A_Init(void);  
  
  
  
#ifdef __cplusplus
}
#endif
#endif /* DC2039A_H_ */