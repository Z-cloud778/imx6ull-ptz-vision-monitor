/*************************************************
 * @File:step_motor.h
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/

#ifndef __STEP_MOTOR_H
#define __STEP_MOTOR_H

#include "stm32f1xx_hal.h"

// 引脚宏定义，和你CubeMX配置对应
#define IN1_PIN     GPIO_PIN_0
#define IN1_PORT    GPIOB
#define IN2_PIN     GPIO_PIN_1
#define IN2_PORT    GPIOB
#define IN3_PIN     GPIO_PIN_4
#define IN3_PORT    GPIOB
#define IN4_PIN     GPIO_PIN_3
#define IN4_PORT    GPIOB

void StepMotor_SetIO(uint8_t val1,uint8_t val2,uint8_t val3,uint8_t val4);
void StepMotor_Run(uint8_t dir,uint16_t step,uint16_t delay_ms);
void StepMotor_RotateAngle(float angle,uint8_t dir,uint16_t speed_delay);

#endif