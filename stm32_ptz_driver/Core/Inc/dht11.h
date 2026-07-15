/*************************************************
 * @File:dht11.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/

#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f1xx_hal.h"

// 引脚宏定义，和CubeMX配置一致
#define DHT11_PIN    GPIO_PIN_2
#define DHT11_PORT   GPIOA

// 存储温湿度数据
typedef struct
{
    uint8_t temp_int;    // 温度整数部分
    uint8_t temp_dec;    // 温度小数部分(DHT11固定0)
    uint8_t hum_int;     // 湿度整数部分
    uint8_t hum_dec;     // 湿度小数部分(DHT11固定0)
} DHT11_DataTypeDef;

extern DHT11_DataTypeDef dht11_data;

void DHT11_Init(void);
uint8_t DHT11_ReadData(void);

#endif