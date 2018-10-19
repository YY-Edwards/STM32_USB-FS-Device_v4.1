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
	//C�����н����������һ���,����ͷ�ļ���#include <stdarg.h>,���ڻ�ȡ��ȷ�������Ĳ��� 
	va_list vArgList;

	//��va_list�������г�ʼ������vArgListָ��ָ������б��еĵ�һ������
	va_start(vArgList, psz_fmt);

	//��vArgList(ͨ�����ַ���) ��format��ʽд���ַ���string��
	vsnprintf(msg, 256, psz_fmt, vArgList);
	//����vArgListָ��
	va_end(vArgList);

        logger_count++;//��־����
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
                //log_warning("push log err!");�ݹ���ѭ���ˣ�
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