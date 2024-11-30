#include <ctype.h>
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "usb_device.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "Auto.h"
#include "CRC8_CRC16.h"

extern UART_HandleTypeDef huart1;
uint8_t usart1_receive_buf[14]={0};
struct Vision_info_get Vision_info;

bool_t Vision_read_data(uint8_t *ReadFromUsart)
{
    uint16_t data_len;
    if(ReadFromUsart==NULL)
    {
        return 0;
    }
    if(ReadFromUsart[0]==0XA5)
    {
        if(verify_CRC8_check_sum(&ReadFromUsart[1],4))
        {
            Vision_info.data_length.data[0]=ReadFromUsart[1];
            Vision_info.data_length.data[1]=ReadFromUsart[2];
            Vision_info.seq=ReadFromUsart[3];
        }
        Vision_info.frame_header.cmd=ReadFromUsart[4];
        Vision_info.yaw.data[0]=ReadFromUsart[5];
        Vision_info.yaw.data[1]=usart1_receive_buf[6];
        Vision_info.yaw.data[2]=usart1_receive_buf[7];
        Vision_info.yaw.data[3]=usart1_receive_buf[8];
        if(verify_CRC16_check_sum(&ReadFromUsart[9],3))
        {
            Vision_info.target_lock=ReadFromUsart[9]<<8 | ReadFromUsart[10];
        }
    }

}

static void auto_init(){
    //接收结构体
    Vision_info.frame_header.head=0xA5;
    Vision_info.yaw.value=0;
}
int a =0;
void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_FEFLAG(&huart1);

        __HAL_DMA_DISABLE(huart1.hdmarx);

        //Vision_read_data(&usart1_receive_buf[0]);
        a = 1;
        memset(&usart1_receive_buf[0],0,VISION_BUFFER_SIZE);

        __HAL_DMA_CLEAR_FLAG(huart1.hdmarx,DMA_HISR_TCIF5);

        __HAL_DMA_SET_COUNTER(huart1.hdmarx,VISION_BUFFER_SIZE);

        __HAL_DMA_ENABLE(huart1.hdmarx);


    }
}

void Auto_task(void const* pvParameters)
{
    //osDelay(AUTO_TASK_INIT_TIME);
    auto_init();
    while(1)
    {
        //Vision_info.frame_header.cmd=ReadFromUsart[4];
        HAL_UART_Receive(&huart1,usart1_receive_buf,sizeof(usart1_receive_buf),HAL_MAX_DELAY);
//            if(verify_CRC8_check_sum(&usart1_receive_buf[4],4))
//            {
//                Vision_info.data_length.data[0]=usart1_receive_buf[1];
//                Vision_info.data_length.data[1]=usart1_receive_buf[2];
//                Vision_info.seq=usart1_receive_buf[3];
//            }
            Vision_info.frame_header.cmd=usart1_receive_buf[5]<<8 | usart1_receive_buf[6];
            Vision_info.yaw.data[0]=usart1_receive_buf[7];
            Vision_info.yaw.data[1]=usart1_receive_buf[8];
            Vision_info.yaw.data[2]=usart1_receive_buf[9];
            Vision_info.yaw.data[3]=usart1_receive_buf[10];
            Vision_info.target_lock=usart1_receive_buf[12]<<8|usart1_receive_buf[13];
        //memset(&usart1_receive_buf[0],0,VISION_BUFFER_SIZE);
        osDelay(2);
    }
}