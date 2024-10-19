//
// Created by 小新 on 2024/2/18.
//

#ifndef STM32_FREERTOS_DECODE_H
#define STM32_FREERTOS_DECODE_H

#include "cmsis_os.h"
#include "fifo.h"
#include "protocol_shaob.h"
#include "CRC8_CRC16.h"

void decode_unpack_fifo_data();                 // 解包校验数据
uint16_t decode_data_solve(uint8_t *frame);     // 解析数据包

/* CRC校验函数 */
extern uint32_t verify_CRC8_check_sum(unsigned char *pch_message, unsigned int dw_length);
extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
#endif //STM32_FREERTOS_DECODE_H
