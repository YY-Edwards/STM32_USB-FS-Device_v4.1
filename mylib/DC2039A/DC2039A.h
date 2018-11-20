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
#include "task_timer.h"
#include "BCMP.h"  
  
  
#if defined(STM32L1XX_MD)

#define SMBALERT_IN_PIN              PBin(14)
#define U5NWP_OUT_PIN                PBout(8) 
#define DVCC_OUT_PIN                 PBout(12) 
#define DR_ADDRESS                  ((uint32_t)0x40012458) //ADC1 DR¼Ä´æÆ÷»ùµØÖ·

#else

#define SYSTEM_CLOCK 72000000
#define DR_ADDRESS                  ((uint32_t)0x4001244C) //ADC1 DR¼Ä´æÆ÷»ùµØÖ·
#define TP1_OUT_PIN                  PAout(2)   
#define TP2_IN_PIN                   PAin(3)  
#define SMBALERT_IN_PIN              PAin(4)
#define U5NWP_OUT_PIN                PAout(5) 
#define NEQ_OUT_PIN                  PAout(6) 
#define DVCC_OUT_PIN                 PAout(6) 

#endif  
  
  
  
void DC2039A_Init(void);  
  
  
  
#ifdef __cplusplus
}
#endif
#endif /* DC2039A_H_ */