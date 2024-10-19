//
// Created by 小新 on 2024/2/18.
//

#ifndef STM32_FREERTOS_PACKET_H
#define STM32_FREERTOS_PACKET_H

#include <memory.h>
#include "stdint.h"
#include "cmsis_os.h"
#include "protocol_shaob.h"
#include "CRC8_CRC16.h"

extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len);                      // 添加数据进入队列
extern void rm_dequeue_send_data(void* buf,uint16_t len);                               // 从队列中取出数据进行序列化
extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);    // 添加 CRC8 校验值在一段数据的结尾
extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);             // 添加 CRC16 校验值在一段数据的结尾
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);                             // USB底层发送函数

static void encode_send_data(uint16_t cmd_id,void* buf ,uint16_t len);                  // 数据序列化

#endif //STM32_FREERTOS_PACKET_H
