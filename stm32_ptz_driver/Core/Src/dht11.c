/*************************************************
 * @File: dht11.c
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/


#include "dht11.h"

DHT11_DataTypeDef dht11_data;

// 简单微秒级延时（72M系统时钟）
static void DHT11_DelayUs(uint32_t us)
{
    uint32_t i;
    for(i = 0; i < us * 7; i++)
    {
        __NOP();
    }
}

// 设置引脚为输出模式
static void DHT11_PIN_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

// 设置引脚为输入模式
static void DHT11_PIN_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

// DHT11初始化
void DHT11_Init(void)
{
    DHT11_PIN_OUT();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    HAL_Delay(100);
}

// 读取单字节数据
static uint8_t DHT11_ReadByte(void)
{
    uint8_t data = 0;
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        // 等待低电平起始
        while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET);
        DHT11_DelayUs(30);
        if(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET)
        {
            data |= (1 << (7 - i));
        }
        // 等待高电平结束
        while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET);
    }
    return data;
}

// 读取一次温湿度，返回0成功，1校验失败
uint8_t DHT11_ReadData(void)
{
    uint8_t hum, hum_dec, temp, temp_dec, check_sum;

    // 主机拉低18ms发起起始信号
    DHT11_PIN_OUT();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(18);
    // 拉高20~40us等待DHT11应答
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    DHT11_DelayUs(30);
    // 切换为输入接收应答
    DHT11_PIN_IN();

    // 等待DHT11拉低应答
    while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET);
    while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET);
    while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET);

    // 读取4字节数据+1字节校验
    hum = DHT11_ReadByte();
    hum_dec = DHT11_ReadByte();
    temp = DHT11_ReadByte();
    temp_dec = DHT11_ReadByte();
    check_sum = DHT11_ReadByte();

    // 校验
    if((hum + hum_dec + temp + temp_dec) == check_sum)
    {
        dht11_data.hum_int = hum;
        dht11_data.hum_dec = hum_dec;
        dht11_data.temp_int = temp;
        dht11_data.temp_dec = temp_dec;
        // 读完切回输出拉高
        DHT11_PIN_OUT();
        HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
        return 0;
    }
    DHT11_PIN_OUT();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    return 1;
}