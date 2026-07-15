/*************************************************
 * @File: step_motor.c
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/

#include "step_motor.h"

// 设置四路IO电平
void StepMotor_SetIO(uint8_t val1,uint8_t val2,uint8_t val3,uint8_t val4)
{
    HAL_GPIO_WritePin(IN1_PORT,IN1_PIN,val1?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_PORT,IN2_PIN,val2?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_PORT,IN3_PIN,val3?GPIO_PIN_SET:GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_PORT,IN4_PIN,val4?GPIO_PIN_SET:GPIO_PIN_RESET);
}

// 八拍时序表
uint8_t step_table[8][4]={
    {0, 0, 1, 0},  // 第0拍：C相单独通电
    {0, 0, 1, 1},  // 第1拍：C+D两相通电
    {0, 0, 0, 1},  // 第2拍：D相单独通电
    {1, 0, 0, 1},  // 第3拍：A+D两相通电
    {1, 0, 0, 0},  // 第4拍：A相单独通电
    {1, 1, 0, 0},  // 第5拍：A+B两相通电
    {0, 1, 0, 0},  // 第6拍：B相单独通电
    {0, 1, 1, 0},  // 第7拍：B+C两相通电

};

/**
 * @brief 步进电机运行
 * @param dir 0逆时针 1顺时针
 * @param step 总步数
 * @param delay_ms 每拍延时(ms)，越小速度越快
 */
void StepMotor_Run(uint8_t dir,uint16_t step,uint16_t delay_ms)
{
    uint16_t i,j;
    uint8_t index=0;
    for(i=0;i<step;i++)
    {
        if(dir == 1) // 顺时针
        {
            index++;
            if(index>=8) index=0;
        }
        else          // 逆时针
        {
            index--;
            if(index>=8) index=7;
        }
        StepMotor_SetIO(step_table[index][0],step_table[index][1],
                        step_table[index][2],step_table[index][3]);
        HAL_Delay(delay_ms);
    }
    StepMotor_SetIO(0,0,0,0); // 转动结束断电，防止电机发热
}

/**
 * @brief 指定角度旋转
 * @param angle 旋转角度
 * @param dir 1顺时针 0逆时针
 * @param speed_delay 每拍延时ms
 * 28YJJ-48：步距角5.625° 减速比1:64，一圈总步数=360/5.625*64=4096步
 */
void StepMotor_RotateAngle(float angle,uint8_t dir,uint16_t speed_delay)
{
    uint32_t total_step = angle * 4096 / 360.0f;
    StepMotor_Run(dir,total_step,speed_delay);
}