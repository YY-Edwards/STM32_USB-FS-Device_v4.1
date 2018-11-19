#ifndef TASK_TIMER_H_
#define TASK_TIMER_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "platform_config.h"  
#include "stdbool.h"
#include <string.h>  

typedef void (*handler)(void*);  

void task_reset();
volatile void timer_task_schedule_func(void);
void task_process();
void task_init();
void start_timer_task_schedule();
void stop_timer_task_schedule();
void start_bnp_timeout_detect();
void stop_bnp_timeout_detect(void);


void set_timer_task(unsigned char       timer_id, 
                    unsigned int        delay, 
                    unsigned char       rearm, 
                    handler             timehandler, 
                    void *              param );
 
 
#ifdef __cplusplus
}
#endif
#endif /* TASK_TIMER_H_ */
