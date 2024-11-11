#include <ctype.h>
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "usb_device.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

uint8_t buffer[6];
float value;


void lali_task(void const* pvParameters)
{
    while(1) {
        HAL_UART_Receive(&huart1, buffer, 6, HAL_MAX_DELAY);
        uint8_t xiaoshu = 0, zhengshu = 0, flag = 0;
        for (uint8_t i = 0; i < 6; i++) {
            if (buffer[i] == '.') {
                flag = 1;
                continue;
            }
            if (flag == 0) {
                uint8_t cnt = buffer[i] - '0';
                zhengshu = zhengshu * 10 + cnt;
            } else {
                uint8_t cnt = buffer[i] - '0';
                xiaoshu = xiaoshu * 10 + cnt;
            }
        }
        value = zhengshu + (float) xiaoshu / 1000;
        vTaskDelay(10);
    }
}