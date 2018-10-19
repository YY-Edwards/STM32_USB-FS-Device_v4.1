#include "logger.h"


RingQueue_t logger_msg_queue_ptr = NULL;
static unsigned int logger_count = 0;

void logger_init()
{
  
  if(logger_msg_queue_ptr!=NULL)
  {
     free(logger_msg_queue_ptr);
     logger_msg_queue_ptr =NULL;
  }
  
  logger_msg_queue_ptr = malloc(sizeof(ring_queue_t));
  if(logger_msg_queue_ptr ==NULL)
  {
    //printf("malloc ble_msg_queue_ptr failure\r\n");
    return ;
  }
  
  init_queue(logger_msg_queue_ptr);

}

void logger_add_msg_to_queue(const char* psz_level,
                                const char* psz_file,
                                int line_no,
                                const char* psz_funcsig,
                                char *psz_fmt, ...)
{

  	char msg[256] = { 0 };
	//C语言中解决变参问题的一组宏,所在头文件：#include <stdarg.h>,用于获取不确定个数的参数 
	va_list vArgList;

	//对va_list变量进行初始化，将vArgList指针指向参数列表中的第一个参数
	va_start(vArgList, psz_fmt);

	//将vArgList(通常是字符串) 按format格式写入字符串string中
	vsnprintf(msg, 256, psz_fmt, vArgList);
	//回收vArgList指针
	va_end(vArgList);

        logger_count++;//日志计数
        if(logger_count == 0xffffffff)
        {
          logger_count = 0;
        }
	char content[280] = { 0 };
	sprintf(content,
		//"[%d][%s][%s:%d %s]: %s\r\n",
                "[%d][%s][%s:%d]: %s\r\n",
		logger_count,
		psz_level,
                //psz_file,
		//line_no,
		psz_funcsig,
                line_no,
		msg);



	{
            if(logger_msg_queue_ptr !=NULL)
            {
              bool ret =false;
              ret = push_to_queue(logger_msg_queue_ptr, content, (strlen(content)+1));
              if(ret !=true)
              {
                memset(content, 0x00, sizeof(content));
                //log_warning("push log err!");递归死循环了？
              }
            }
	
	}

}

bool logger_output_msg(void *buf, unsigned short *buf_len)
{
  bool ret =false;
  if(logger_msg_queue_ptr !=NULL)
  {
     ret = take_from_queue(logger_msg_queue_ptr, buf, (int *)buf_len, true);
  }
  return ret;
}