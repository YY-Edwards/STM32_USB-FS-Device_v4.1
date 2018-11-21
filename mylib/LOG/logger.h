#ifndef _LOGGER_H_
#define _LOGGER_H_
#include "myqueue.h"
#include "platform_config.h"
#include "task_timer.h"
#include "stdio.h"
#include <stdarg.h>

#define log_debug(...)	logger_add_msg_to_queue("DEBUG", __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)
#define log_info(...)	logger_add_msg_to_queue("INFO", __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)
#define log_warning(...) logger_add_msg_to_queue("WARNING", __FILE__, __LINE__, __PRETTY_FUNCTION__, __VA_ARGS__)

void logger_init();
void logger_add_msg_to_queue(const char* psz_level,
                                const char* psz_file,
                                int line_no,
                                const char* psz_funcsig,
                                const char *psz_fmt, ...);
extern bool logger_output_msg(void *buf, unsigned short *buf_len);
extern void my_dma_config_and_enabled(void *p, uint16_t p_len);

#endif