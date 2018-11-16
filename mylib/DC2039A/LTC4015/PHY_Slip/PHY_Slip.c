#include "PHY_Slip.h"
#include "usb_conf.h"
#include "usb_regs.h"
#include "usb_desc.h"

volatile RingQueue_t usb_rx_queue_ptr = NULL;
volatile RingQueue_t  slip_usb_send_queue_ptr = NULL;


static phy_fragment_t g_rx_bnp_frame;
extern volatile RingQueue_t bnp_rx_queue_ptr;
extern volatile RingQueue_t bnp_tx_queue_ptr;
unsigned char ptrBuffer[64];
__IO uint32_t packet_sent = 1;
extern void UserToPMABufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);

void Handle_USBAsynchXfer (void)
{
    //VCP_SendRxBufPacketToUsb();
  if(packet_sent == 1)
  {   
      int len = 0;
      bool ret = take_from_queue(slip_usb_send_queue_ptr, &ptrBuffer[0], &len, true);
      if(ret == true)//recv and assemble
      {      
        /* Reset Sent flag*/
        packet_sent = 0;
        /* send  packet to PMA*/
        UserToPMABufferCopy((unsigned char*)ptrBuffer, ENDP1_TXADDR, len);
        SetEPTxCount(ENDP1, len);//从端点1发送Send_length字节数据
        SetEPTxValid(ENDP1);//使能端点1的发送状态
      }
  }
  
}

void phy_slip_assemble_task(void *p)
{
  
  static SLIP_parser_state_enum m_state = SLIP_FIND_HEADER;
  static int slip_recv_msg_idx = 0;
  
  int recv_len =0;
  uint8_t g_usb_buf[256]={0};
  memset(g_usb_buf, 0x00, sizeof(g_usb_buf));
  
  bool ret = take_from_queue(usb_rx_queue_ptr, &g_usb_buf[0], &recv_len, true);
  if(ret == true)//recv and assemble
  {
    log_debug("usb rx slip len: [%d] ", recv_len);
                
    int bnp_package_len = 0;
    int index =0;
    uint8_t ch =0;
    uint8_t nextch =0;
    while(0 < recv_len--)//字节流解析
    {
      ch = g_usb_buf[index++];//从buffer中取出一个字节的数据，进行解析
      switch(m_state)
      {
        case SLIP_FIND_HEADER:
            if(ch == END)
            {
              slip_recv_msg_idx = 0;//reset index
              m_state = SLIP_READ_DATA;      
            }
            break;
          
        case SLIP_READ_DATA:
            if(ch == ESC)
            {
              nextch = g_usb_buf[index++];//从buffer中取出下一个字节的数据，进行解析
              if(nextch == ESC_END)//转译
              {
                g_rx_bnp_frame.u8[slip_recv_msg_idx++] = END;
              }
              else  if(nextch == ESC_ESC)//转译
              {
                g_rx_bnp_frame.u8[slip_recv_msg_idx++] = ESC;
              }
              else//不符合协议
              {
                log_warning("slip erron,[nextch]:0x%x",nextch);
                m_state = SLIP_FIND_HEADER;//丢弃之前收到数据，继续查询g_usb_buf，直到找到END
                
              }
              
            }
            else if(ch == END)//结束,推送完整的bnp包
            {            
              m_state = SLIP_FIND_HEADER;
              bnp_package_len = slip_recv_msg_idx+1;
              
              log_debug("slip rx okay, slip_len: [%d], bnp_len:[%d]", index, bnp_package_len);
              
              ret = push_to_queue(bnp_rx_queue_ptr, g_rx_bnp_frame.u8, bnp_package_len);
              if(ret != true)
              {
                log_warning("bnp_rx_queue_ptr full!");          
              } 
            }
            else
            {
              g_rx_bnp_frame.u8[slip_recv_msg_idx++] = ch;
            }
              
            break;
          
        
        default:   
            //no should happen,but must check
            log_warning("slip parse state erron!");
            break;
       
      }
    }
  }
  else
  {
    bool ret = take_from_queue(bnp_tx_queue_ptr, &g_usb_buf[0], &recv_len, true);
    if(ret == true)//assemble and send
    {
        uint16_t index =0;
        uint16_t idx =0;
        uint8_t usb_send_buf[256];
        memset(usb_send_buf, 0x00, sizeof(usb_send_buf));
        
        usb_send_buf[idx++] = END;
	while (recv_len-- > 0)
	{
            switch (*((uint8_t *)g_usb_buf + index))//从g_usb_buf取出一个字节的数据
            {
            case END:
                    usb_send_buf[idx++] = ESC;
                    usb_send_buf[idx++] = ESC_END;
                    break;

            case ESC:
                    usb_send_buf[idx++] = ESC;
                    usb_send_buf[idx++] = ESC_ESC;
                    break;
            default:
                    usb_send_buf[idx++] = (*((uint8_t *)g_usb_buf + index));
                    break;
            }
            
            index++;
	}
	usb_send_buf[idx++] = END; 
            
        unsigned short remained_bytes = idx;
        do
        {
          if(remained_bytes > VIRTUAL_COM_PORT_DATA_SIZE) 
          {
            ret = push_to_queue(slip_usb_send_queue_ptr, usb_send_buf, VIRTUAL_COM_PORT_DATA_SIZE);
            if(ret != true)
            {
              log_warning("slip_usb_send_queue_ptr full!");          
            }
            remained_bytes -= VIRTUAL_COM_PORT_DATA_SIZE;
          }
          else
          {
            ret = push_to_queue(slip_usb_send_queue_ptr, usb_send_buf, remained_bytes);             
            if(ret != true)
            {
              log_warning("slip_usb_send_queue_ptr full!");          
            }
            remained_bytes = 0;
          }
          
        }while(remained_bytes > 0);

    }  
  }
}

void phy_slip_init()
{
  usb_rx_queue_ptr = create_queue(20, 64);
  if(usb_rx_queue_ptr == NULL)
  {
    return;
  }
  
  slip_usb_send_queue_ptr = create_queue(20, 64);
  if(slip_usb_send_queue_ptr == NULL)
  {
    return;
  }
   
  set_timer_task(SLIP_ASSEMBLE_TASK, 3*TIME_BASE_10MS, true, phy_slip_assemble_task, NULL);
  
}