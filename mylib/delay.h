#ifndef __DELAY_H
#define __DELAY_H 			   
#if defined (USE_STM32L152_EVAL)
#include "stm32l1xx.h"
#else
#include <stm32f10x.h>
#endif
//#include "stm32f10x.h"

void delay_init(uint8_t SYSCLK);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
void Delay(uint32_t nCount);
#endif





























