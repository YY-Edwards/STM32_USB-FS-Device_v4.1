#include "myqueue.h"


bool init_queue(RingQueue_t ring_queue)
{
  ring_queue->head = 0;
  ring_queue->tail = 0;
  
  unsigned short array_deep = ring_queue->queue_deep;
  unsigned short data_size = ring_queue->data_size;
  
  ring_queue->queue_point = malloc(array_deep);//分配队列深度内存
  if(ring_queue->queue_point == NULL)return false;
  for(int i =0; i < (ring_queue->queue_deep); i++)
  {
    (ring_queue->queue_point +i)->data = malloc(data_size);//分配队列宽度内存
    if((ring_queue->queue_point +i)->data ==NULL)return false;
    memset((ring_queue->queue_point +i)->data, 0x00, data_size);//清理数据
    ring_queue->queue_point->len=0;
  }
  return true;
}



bool take_from_queue(RingQueue_t ring_queue, void *buf, int *len, bool erase)
{
  bool ret =false;
  if(ring_queue == NULL)return false;
  
  int snap_head = ring_queue->head;
  if(snap_head != ring_queue->tail)
  {
    memcpy(buf, ring_queue->queue_point[ring_queue->tail].data,  ring_queue->queue_point[ring_queue->tail].len);
    *len = ring_queue->queue_point[ring_queue->tail].len;
    if(true == erase)
    {
      ring_queue->tail= ring_queue->tail + 1;
      if(ring_queue->tail == ring_queue->queue_deep)
      {
        ring_queue->tail = 0;
      }
    }
    ret = true;//success
  }
  else
  {
    ret = false;//empty
  }
  
  return ret;
}
bool push_to_queue(RingQueue_t ring_queue, void *buf, int len)
{
  dyn_mydata_t *p =NULL;
  int next_index =0;
  bool ret =false;
  if(len == 0)return false;
  if(ring_queue == NULL)return false;
  
  if(len > ring_queue->data_size)
     len = ring_queue->data_size;
  
 // p = (dyn_mydata_t *)(&(ring_queue->queue_point[ring_queue->head]));
   p = (dyn_mydata_t *)((ring_queue->queue_point + ring_queue->head));
  memcpy(p->data, buf, len);
  p->len = len;
  
  next_index = ring_queue->head +1;
  if(next_index != ring_queue->tail)
  {
    if(next_index == ring_queue->queue_deep)
    {
      next_index =0 ;
    }
    ring_queue->head = next_index;
    ret = true;
    
  }
  else
  {
    ret = false;
  }
  
  return ret;

}
void clear_queue(RingQueue_t ring_queue)
{
  ring_queue->head = ring_queue->tail;
}
bool queue_is_empty(RingQueue_t ring_queue)
{
  if(ring_queue == NULL)return false;
  
  int mid_flag = ring_queue->head;
  if(mid_flag != ring_queue->tail)
  {
    return false;
  }
  else
  {
    return true;
  }
}



Queue_t QueueCreate(unsigned short deep, unsigned short elementsize )
{    
    if(( 0 == deep) || ( 0 == elementsize ))
    {
        return NULL;
    }
    
    Queue_t pQueue = (Queue_t)malloc(sizeof(QueueStr_t));
    if(NULL == pQueue)
    {
        return NULL;
    }
    
    pQueue->store = malloc(deep * elementsize);
    if(NULL == pQueue->store)
    {
        free(pQueue);
        return NULL;
    }
    
    pQueue->deep = deep;
    pQueue->elementsize = elementsize;
    pQueue->front = 0;
    pQueue->rear = 0;
    pQueue->count = 0;
    
    return pQueue;
}

QueueSta_t QueueClear(Queue_t queue)
{
  if(NULL == queue) return queue_null;
  

    queue->front = 0;
    queue->rear = 0;
    queue->count = 0;
    return queue_ok;
}

QueueSta_t QueueDelete(Queue_t queue)
{
    if(NULL == queue) return queue_null;
    if(NULL == queue->store)
    {
       free(queue);
       queue = NULL;
       return queue_null;
    }
    else
    {
        free(queue->store);
        free(queue);
        queue = NULL;
        return queue_ok; 
    }
}

QueueSta_t QueuePush(Queue_t queue, void * element)
{
    if(NULL == element) return element_null;
    if(NULL == queue) return queue_null;
    if(NULL == queue->store) return queue_null;
    
    if(queue->count > queue->deep)return queue_full;
    
    unsigned short cur = 0;
    
    if(0 == queue->count)
    {
       cur = 0; 
       queue->rear = 0;
       queue->front = 0;
    }
    else
    {
       cur = (queue->rear + 1 >= queue->deep) ? 0 : (queue->rear + 1);
    }
    
    
    memcpy(queue->store + queue->elementsize * cur, element, queue->elementsize);
    
    queue->rear = cur;
    queue->count += 1;
    
    if(queue->count >= queue->deep)
    {
       return queue_full;
       //queue->count = 0;
       //printf("qxxxqqqqq\r\n");
        //queue->count = 0;
    }
    
    return queue_ok;
}

QueueSta_t QueuePull(Queue_t queue, void * element)
{
    if(NULL == element) return element_null;
    if(NULL == queue) return queue_null;
    if(NULL == queue->store) return queue_null;
    
    if(0 >= queue->count)return queue_empty;
    memcpy(element, queue->store + queue->elementsize * queue->front,  queue->elementsize);
    
    queue->front = (queue->front + 1 >= queue->deep) ? 0 : (queue->front + 1);
    queue->count -= 1;
    
    if(0 >= queue->count)
    {
       QueueClear(queue);
    }
    
    return queue_ok;
}
