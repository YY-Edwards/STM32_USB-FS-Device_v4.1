#include "PHY_Slip.h"
#include "usb_conf.h"
#include "usb_regs.h"
#include "usb_desc.h"

volatile RingQueue_t  usb_rx_queue_ptr = NULL;
volatile RingQueue_t  slip_usb_send_queue_ptr = NULL;


static phy_fragment_t g_rx_bnp_frame;
extern volatile RingQueue_t bnp_rx_queue_ptr;
extern volatile RingQueue_t bnp_tx_queue_ptr;
unsigned char ptrBuffer[64];
__IO uint32_t packet_sent = 1;
extern void UserToPMABufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
extern volatile bool bnp_rx_response_flag;;
volatile bool timeout_for_once = false;

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
        SetEPTxCount(ENDP1, len);//�Ӷ˵�1����Send_length�ֽ�����
        SetEPTxValid(ENDP1);//ʹ�ܶ˵�1�ķ���״̬
      }
  }
  
}

void phy_slip_assemble_task(void *p)
{
  
  int recv_len =0;
  uint8_t rx_usb_buf[256]={0};
  memset(rx_usb_buf, 0x00, sizeof(rx_usb_buf));
  
  bool ret = take_from_queue(usb_rx_queue_ptr, &rx_usb_buf[0], &recv_len, true);
  if(ret == true)//recv and assemble
  {
    log_debug("usb rx slip len: [%d] ", recv_len);
    
    static SLIP_parser_state_enum m_state = SLIP_FIND_HEADER;
    static int slip_recv_msg_idx = 0;
    static int byte_count = 0;
    int bnp_package_len = 0;
    int index =0;
    uint8_t ch =0;
    uint8_t nextch =0;
    while(0 < recv_len--)//�ֽ�������
    {
      byte_count++;
      ch = rx_usb_buf[index++];//��buffer��ȡ��һ���ֽڵ����ݣ����н���
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
              byte_count++;
              nextch = rx_usb_buf[index++];//��buffer��ȡ����һ���ֽڵ����ݣ����н���
              if(nextch == ESC_END)//ת��
              {
                g_rx_bnp_frame.u8[slip_recv_msg_idx++] = END;
              }
              else  if(nextch == ESC_ESC)//ת��
              {
                g_rx_bnp_frame.u8[slip_recv_msg_idx++] = ESC;
              }
              else//������Э��
              {
                log_warning("slip erron,[nextch]:0x%x",nextch);
                m_state = SLIP_FIND_HEADER;//����֮ǰ�յ����ݣ�������ѯrx_usb_buf��ֱ���ҵ�END
                
              }
              
            }
            else if(ch == END)//����,����������bnp��
            {            
              m_state = SLIP_FIND_HEADER;
              bnp_package_len = slip_recv_msg_idx;
              
              log_debug("slip rx okay, slip_len: [%d], bnp_len:[%d]", byte_count, slip_recv_msg_idx);
              
              byte_count = 0;//reset 
              
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
    static custom_data_send_state_enum custom_send_state = CUSTOM_ASSEMBLE_DATA;
    static unsigned char resend_count = 0;
    static uint8_t temp_buf[200]={0};
    static uint8_t usb_send_buf[256]={0};
    static uint16_t idx =0;
    static custom_bnp_send_data_t *ptr = (custom_bnp_send_data_t *)temp_buf;//ע�⣬ָ���Ƿ�Խ��
    
    switch(custom_send_state)
    {
      
      case CUSTOM_ASSEMBLE_DATA:
          {
              memset(temp_buf, 0x00, sizeof(temp_buf));//clear buf
              bool ret = take_from_queue(bnp_tx_queue_ptr, (void*)&temp_buf[0], &recv_len, true);
              if(ret == true)//assemble
              {               
                  log_debug("bnp send a msg.");
                  uint16_t index =0;
                  
                  idx =0;
                  memset(usb_send_buf, 0x00, sizeof(usb_send_buf));
                  unsigned short bnp_len = recv_len-4;
                  usb_send_buf[idx++] = END;
                  while (bnp_len-- > 0)
                  {
                      switch (*((uint8_t *)(ptr->phy_valid_data.u8) + index))//��temp_bufȡ��һ���ֽڵ�����
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
                              usb_send_buf[idx++] = (*((uint8_t *)(ptr->phy_valid_data.u8) + index));
                              break;
                      }
                      
                      index++;
                  }
                  usb_send_buf[idx++] = END; 
                  custom_send_state = CUSTOM_WAIT_FOR_TX;
                  resend_count = 0;//reset count
              }
          }
        
          break;
      case CUSTOM_WAIT_FOR_TX:
          {         
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
            
            if(ptr->is_data_need_answer == 1)
            {
              custom_send_state = CUSTOM_WAIT_RESPONSE;
              start_bnp_timeout_detect();//����bnp response���
            }
            else//no need wait for response
            {
              custom_send_state = CUSTOM_ASSEMBLE_DATA;
            }
          }
           
        break;
      case CUSTOM_WAIT_RESPONSE:
          if(bnp_rx_response_flag == true)
          {
            bnp_rx_response_flag = false;//reset
            custom_send_state = CUSTOM_ASSEMBLE_DATA;
            stop_bnp_timeout_detect();//�ر�bnp response���
          }
          else//�ȴ��豸��Ӧ
          {
            if(resend_count <= PHY_MAX_RESEND_COUNTS)
            {
              if(timeout_for_once == true)//��ʱһ��
              {
                timeout_for_once = false;
                resend_count++;
                log_warning("device timeout :[%d].", resend_count);   
                custom_send_state = CUSTOM_WAIT_FOR_TX;//resend bnp
              }          
            }
            else
            {
              log_warning("device no response!");
              timeout_for_once = false;
              resend_count = 0;
              stop_bnp_timeout_detect();//�ر�bnp response���
              custom_send_state = CUSTOM_ASSEMBLE_DATA;
            }         
          }
             
          break;
      default:
        break;
    } 
  }
}

void phy_slip_init()
{
  usb_rx_queue_ptr = create_queue(10, 64);
  if(usb_rx_queue_ptr == NULL)
  {
    return;
  }
  
  slip_usb_send_queue_ptr = create_queue(10, 64);
  if(slip_usb_send_queue_ptr == NULL)
  {
    return;
  }
  
  bnp_tx_queue_ptr = create_queue(5, sizeof(custom_bnp_send_data_t));
  if(bnp_tx_queue_ptr == NULL)
  {
    return;
  }
   
  set_timer_task(SLIP_ASSEMBLE_TASK, 3*TIME_BASE_10MS, true, phy_slip_assemble_task, NULL);
  
}