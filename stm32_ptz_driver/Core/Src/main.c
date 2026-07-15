/*************************************************
 * @File: main.c
 * @Author: Z-cloud778
 * @Date: 2026-07-14
*************************************************/


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
  
#include "step_motor.h"
#include "dht11.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t uart_rx_byte;

char uart_rx_buf[64];
uint8_t uart_rx_idx = 0;

char uart_cmd_buf[64];
volatile uint8_t uart_cmd_ready = 0;

int servo_h_angle = 90;
int servo_v_angle = 90;

/*
 * 新增：舵机目标角度
 * servo_h_angle / servo_v_angle 表示当前实际输出角度
 * servo_h_target_angle / servo_v_target_angle 表示上位机要求到达的目标角度
 */
int servo_h_target_angle = 90;
int servo_v_target_angle = 90;

/*
 * 舵机平滑控制参数
 * 每 20ms 最多移动 1°
 */
#define SERVO_SMOOTH_PERIOD_MS   20
#define SERVO_SMOOTH_STEP        1

// 新增DHT11采集时间戳
uint32_t last_dht11_tick = 0;


/*
pir_motion_event：中断里置 1
pir_last_trigger_tick：最近一次检测到人体运动的时间
pir_hold_until_tick：保持 PIR 有人的时间
PIR_HOLD_MS：检测到运动后至少保持 8 秒有人状态
*/


volatile uint8_t pir_motion_event = 0;

uint8_t pir_reported_active = 0;
uint32_t pir_last_trigger_tick = 0;
uint32_t pir_hold_until_tick = 0;

#define PIR_HOLD_MS 8000

#define PIR_HIGH_CONFIRM_MS   500
#define PIR_HOLD_MS           8000


/*

host_sleep_armed = 1：
说明 Linux 已经进入或即将进入系统级休眠，
此时 PIR 触发时，STM32 才需要拉高 WAKE_OUT。

host_sleep_armed = 0：
说明 Linux 正常运行，
PIR 只需要串口上报 PIR,1，不需要拉高 WAKE_OUT。

*/
uint8_t host_sleep_armed = 0;

#define PIR_SLEEP_DENY_MS 5000

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf 重定向到 USART1 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);
    return ch;
}

static int clamp_int(int value, int min, int max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ===================== MQ2 参数 =====================

uint8_t mq2_alarm_state = 0;

#define MQ2_ADC_ALARM_TH      1950
#define MQ2_ADC_RECOVER_TH    1850


#define MQ2_ADC_MAX        4095.0f
#define MQ2_ADC_VREF       3.3f

/*
 * MQ2 传感器测试电压。
 * 你的 MQ2 模块一般 VCC 接 5V，所以这里写 5.0。
 */
#define MQ2_VC             5.0f

/*
 * 负载电阻 RL，单位 kΩ。
 *
 * 如果你真的把 RL 调成 500Ω，就写 0.5f。
 * 如果你的模块实际是 10kΩ，就写 10.0f。
 *
 * 先不要盲目写 0.5，要以你万用表测量为准。
 */
#define MQ2_RL_KOHM        0.7f

/*
 * R0 标定值，单位 kΩ。
 * 这里先给一个临时值。
 * 后面用正常空气下的 ADC 自动计算出来再替换。
 */
static float mq2_r0_kohm = 1.80f;

/*
 * ADC 转电压
 */
static float MQ2_ADC_ToVoltage(uint16_t adc)
{
    return ((float)adc) * MQ2_ADC_VREF / MQ2_ADC_MAX;
}

/*
 * 根据 Vrl 计算 Rs，单位 kΩ
 *
 * Rs = (Vc - Vrl) / Vrl * RL
 */
static float MQ2_CalcRs(float vrl)
{
    if (vrl <= 0.01f)
        return -1.0f;

    return (MQ2_VC - vrl) / vrl * MQ2_RL_KOHM;
}

/*
 * 按你发的文章公式估算 ppm。
 *
 * 注意：这个值只能作为估算值，不是精密浓度。
 */
static float MQ2_CalcPPM_ByArticle(float rs)
{
    if (rs <= 0.0f)
        return 0.0f;

    return powf(11.5428f * mq2_r0_kohm / rs, 0.6549f) * 100.0f;
}

/*
 * 多次采样求平均，减少 ADC 抖动
 */
static uint16_t MQ2_ReadAdcAverage(uint8_t times)
{
    uint32_t sum = 0;

    if (times == 0)
        times = 1;

    for (uint8_t i = 0; i < times; i++)
    {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        sum += HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        HAL_Delay(5);
    }

    return (uint16_t)(sum / times);
}

static float MQ2_CalcR0_InCleanAir(uint16_t adc)
{
    float v = MQ2_ADC_ToVoltage(adc);
    float rs = MQ2_CalcRs(v);

    /*
     * 按文章的做法：
     * 正常空气下 R0 直接取当前 Rs。
     */
    return rs;
}

static void MQ2_UpdateAlarm(uint16_t adc_val)
{
    if (mq2_alarm_state == 0)
    {
        if (adc_val >= MQ2_ADC_ALARM_TH)
        {
            mq2_alarm_state = 1;
        }
    }
    else
    {
        if (adc_val <= MQ2_ADC_RECOVER_TH)
        {
            mq2_alarm_state = 0;
        }
    }
}



/*
 * 舵机角度转 PWM 脉宽
 *
 * 推荐先用 1000us ~ 2000us，比较安全：
 * 0°   -> 1000us
 * 90°  -> 1500us
 * 180° -> 2000us
 *
 * 如果你的舵机支持更大范围，再改成 500~2500us。
 */
static uint16_t Servo_AngleToPulse(int angle)
{
    angle = clamp_int(angle, 0, 180);
    // 0°:500μs  90°:1500μs  180°:2500μs
    return 500 + (uint16_t)((2000U * angle) / 180U);
}

/*
 * channel = 0 -> TIM2_CH1 -> PA0
 * channel = 1 -> TIM2_CH2 -> PA1
 */
void Servo_SetAngle(uint8_t channel, int angle)
{
    uint16_t pulse = Servo_AngleToPulse(angle);

    if (channel == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
    }
    else if (channel == 1)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
    }
}

static int approach_int(int current, int target, int step)
{
    if (step <= 0)
        step = 1;

    if (current < target)
    {
        current += step;

        if (current > target)
            current = target;
    }
    else if (current > target)
    {
        current -= step;

        if (current < target)
            current = target;
    }

    return current;
}

void Servo_SmoothUpdate(void)
{
    int old_h = servo_h_angle;
    int old_v = servo_v_angle;

    servo_h_angle = approach_int(servo_h_angle,
                                 servo_h_target_angle,
                                 SERVO_SMOOTH_STEP);

    servo_v_angle = approach_int(servo_v_angle,
                                 servo_v_target_angle,
                                 SERVO_SMOOTH_STEP);

    if (servo_h_angle != old_h)
    {
        Servo_SetAngle(0, servo_h_angle);
    }

    if (servo_v_angle != old_v)
    {
        Servo_SetAngle(1, servo_v_angle);
    }

    if (servo_h_angle != old_h || servo_v_angle != old_v)
    {
        /* printf("SMOOTH CUR:%d,%d TARGET:%d,%d\r\n",
         *       servo_h_angle,
         *      servo_v_angle,
         *       servo_h_target_angle,
         *      servo_v_target_angle);
		*/
    }
}

/*
 * 串口命令格式：
 *
 * S,90,90
 * S,120,80
 * s,60,130
 *
 * 注意串口助手需要发送换行符：
 * LF 或 CRLF 都可以。
 */
void Process_Uart_Command(char *cmd)
{
    int h = 0;
    int v = 0;
	float angle = 0.0f;
    uint8_t dir = 0;
    uint16_t speed_delay = 2;
	printf("收到指令：%s\r\n", cmd);


    // 舵机控制指令 S,水平,垂直
	if (sscanf(cmd, "S,%d,%d", &h, &v) == 2 ||
		sscanf(cmd, "s,%d,%d", &h, &v) == 2)
	{
		h = clamp_int(h, 0, 180);
		v = clamp_int(v, 0, 180);

		servo_h_target_angle = h;
		servo_v_target_angle = v;

		printf("ACK:TARGET,%d,%d CUR:%d,%d\r\n",
			servo_h_target_angle,
			servo_v_target_angle,
			servo_h_angle,
			servo_v_angle);
	}
    // 步进电机控制指令 M,角度,方向,速度
    else if (sscanf(cmd, "M,%f,%hhu,%hu", &angle, &dir, &speed_delay) == 3 ||
             sscanf(cmd, "m,%f,%hhu,%hu", &angle, &dir, &speed_delay) == 3)
    {
        // 角度范围限制0~360，速度限制1~10ms防止丢步
        if(angle < 0) angle = 0;
        if(angle > 360) angle = 360;
        if(speed_delay < 1) speed_delay = 1;
        if(speed_delay > 10) speed_delay = 10;

        StepMotor_RotateAngle(angle, dir, speed_delay);
        printf("步进电机：%.1f° 方向=%d 单步延时=%dms 执行完成\r\n", angle, dir, speed_delay);
    }
	
	else if(strcmp(cmd, "CAL") == 0 || strcmp(cmd, "cal") == 0)
	{
		uint16_t adc_val = MQ2_ReadAdcAverage(20);
		float v = MQ2_ADC_ToVoltage(adc_val);
		float rs = MQ2_CalcRs(v);

		if (rs > 0.0f)
		{	
			mq2_r0_kohm = rs;

			printf("MQ2 CAL OK ADC:%d V:%.2f R0:%.2f\r\n",
				adc_val,
				v,
				mq2_r0_kohm);
		}
		else
		{
			printf("MQ2 CAL failed!\r\n");
		}
	}
	
	else if(strcmp(cmd,"GET") == 0 || strcmp(cmd,"get") == 0)
	{
		uint16_t adc_val = 0;
		uint32_t now = HAL_GetTick();
		// 必须间隔1s才能再次采集
		if(now - last_dht11_tick < 1000)
		{
			printf("DHT11 busy, wait 1s!\r\n");
			return;
		}

		if(DHT11_ReadData() == 0)
		{
			last_dht11_tick = now;
			adc_val = MQ2_ReadAdcAverage(20);

			float mq2_v = MQ2_ADC_ToVoltage(adc_val);
			float mq2_rs = MQ2_CalcRs(mq2_v);
			float mq2_ppm = MQ2_CalcPPM_ByArticle(mq2_rs);

			MQ2_UpdateAlarm(adc_val);

			printf("T:%d.%d H:%d.%d MQ2_ADC:%d MQ2_V:%.2f MQ2_RS:%.2f MQ2_PPM:%.2f ALARM:%d\r\n",
					dht11_data.temp_int, dht11_data.temp_dec,
					dht11_data.hum_int, dht11_data.hum_dec,
					adc_val,
					mq2_v,
					mq2_rs,
					mq2_ppm,
					mq2_alarm_state);
		}
		else
		{
			printf("DHT11 read error!\r\n");
		}
	}
		
	else if(strcmp(cmd, "GAS") == 0 || strcmp(cmd, "gas") == 0)
	{
		uint16_t adc_val = 0;

		adc_val = MQ2_ReadAdcAverage(5);

		float mq2_v = MQ2_ADC_ToVoltage(adc_val);
		float mq2_rs = MQ2_CalcRs(mq2_v);
		float mq2_ppm = MQ2_CalcPPM_ByArticle(mq2_rs);

		MQ2_UpdateAlarm(adc_val);

		printf("MQ2_ADC:%d MQ2_V:%.2f MQ2_RS:%.2f MQ2_PPM:%.2f ALARM:%d\r\n",
			adc_val,
			mq2_v,
			mq2_rs,
			mq2_ppm,
			mq2_alarm_state);
	}
	
	else if (strcmp(cmd, "HOST,SLEEP_PREPARE") == 0)
    {
        uint32_t now = HAL_GetTick();
		
		printf("收到指令：HOST,SLEEP_PREPARE\r\n");

        /*
         * 如果 PIR 当前就是高电平，或者刚刚触发过，
         * 说明现在不适合让 Linux 休眠。
         */
        if (HAL_GPIO_ReadPin(PIR_OUT_GPIO_Port, PIR_OUT_Pin) == GPIO_PIN_SET ||
            now - pir_last_trigger_tick < PIR_SLEEP_DENY_MS)
        {
            host_sleep_armed = 0;

            HAL_GPIO_WritePin(WAKE_OUT_GPIO_Port,
                              WAKE_OUT_Pin,
                              GPIO_PIN_RESET);

            printf("HOST,SLEEP_DENY\r\n");
        }
        else
        {
            /*
             * 允许 Linux 休眠。
             * 进入休眠前一定要确保 WAKE_OUT 是低电平。
             */	
			host_sleep_armed = 1;
			
            HAL_GPIO_WritePin(WAKE_OUT_GPIO_Port,
                              WAKE_OUT_Pin,
                              GPIO_PIN_RESET);

            

            printf("HOST,SLEEP_READY\r\n");
        }
    }
    else if (strcmp(cmd, "HOST,AWAKE") == 0)
    {
        /*
         * Linux 已经恢复，清除休眠武装状态，
         * 同时释放 WAKE_OUT，避免一直保持高电平。
         */
        host_sleep_armed = 0;

        HAL_GPIO_WritePin(WAKE_OUT_GPIO_Port,
                          WAKE_OUT_Pin,
                          GPIO_PIN_RESET);

        printf("HOST,AWAKE_ACK\r\n");
    }
		

    else
    {
        printf("指令格式错误！\r\n");
        printf("舵机指令示例：S,90,90\r\n");
        printf("步进指令示例：M,90,1,2\r\n");
    }
	
	
	
}


/*
 * USART1 接收中断回调
 *
 * 注意：
 * 这里只接收数据，不 printf，不控制舵机。
 * 收到完整一行后，把命令交给 while(1) 处理。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 仅用 \n 作为帧结束，忽略 \r，解决CRLF两次触发封帧
        if (uart_rx_byte == '\n')
        {
            if (uart_rx_idx > 0)
            {
                uart_rx_buf[uart_rx_idx] = '\0';

                /*
				 * 如果当前没有待处理命令，正常保存。
				 * 如果已经有待处理命令，但新命令是 S/s 舵机命令，
				 * 则允许覆盖旧命令。
				 *
				 * 这样可以保证远程舵机控制优先于 GET 传感器查询。
				 */
				 if (uart_cmd_ready == 0 ||
				 uart_rx_buf[0] == 'S' ||
				 uart_rx_buf[0] == 's')
				 {
					 strcpy(uart_cmd_buf, uart_rx_buf);
					 uart_cmd_ready = 1;
				 }
                // 删掉这里的printf警告
                uart_rx_idx = 0;
                memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
            }
        }
        // 跳过\r回车符，不存入接收缓冲区
        else if (uart_rx_byte != '\r')
        {
            if (uart_rx_idx < sizeof(uart_rx_buf) - 1)
            {
                uart_rx_buf[uart_rx_idx++] = uart_rx_byte;
            }
            else
            {
                // 缓冲区溢出，只重置，删除printf打印
                uart_rx_idx = 0;
                memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
            }
        }

        HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
    }
}

void Print_Clock_Info(void)
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk   = HAL_RCC_GetHCLKFreq();
    uint32_t pclk1  = HAL_RCC_GetPCLK1Freq();
    uint32_t pclk2  = HAL_RCC_GetPCLK2Freq();

    uint32_t tim2clk;

    /*
     * STM32F103 里：
     * TIM2 在 APB1 总线上。
     * 如果 APB1 分频系数不是 1，则 TIM2 时钟 = PCLK1 * 2。
     * 如果 APB1 分频系数是 1，则 TIM2 时钟 = PCLK1。
     */
    if ((RCC->CFGR & RCC_CFGR_PPRE1) == RCC_CFGR_PPRE1_DIV1)
    {
        tim2clk = pclk1;
    }
    else
    {
        tim2clk = pclk1 * 2;
    }

    printf("\r\n===== Clock Info =====\r\n");
    printf("SYSCLK = %lu Hz\r\n", sysclk);
    printf("HCLK   = %lu Hz\r\n", hclk);
    printf("PCLK1  = %lu Hz\r\n", pclk1);
    printf("PCLK2  = %lu Hz\r\n", pclk2);
    printf("TIM2CLK= %lu Hz\r\n", tim2clk);
    printf("======================\r\n");
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PIR_OUT_Pin)
    {
        pir_motion_event = 1;
        pir_last_trigger_tick = HAL_GetTick();
    }
}

void PIR_Task(void)
{
    static uint8_t high_confirming = 0;
    static uint32_t high_start_tick = 0;
    static uint8_t last_pin_state = 0;

    uint32_t now = HAL_GetTick();

    GPIO_PinState pinState =
        HAL_GPIO_ReadPin(PIR_OUT_GPIO_Port, PIR_OUT_Pin);

    uint8_t pir_now = (pinState == GPIO_PIN_SET) ? 1 : 0;

    /*
     * 调试用：只在电平变化时打印。
     * 后面稳定后可以删除。
     */
    if (pir_now != last_pin_state)
    {
        last_pin_state = pir_now;
        printf("PIR_PIN:%d\r\n", pir_now);
    }

    /*
     * 检测到高电平后，不立刻认定有人，
     * 先看它能不能持续保持一段时间。
     */
    if (pir_now)
    {
        if (!high_confirming)
        {
            high_confirming = 1;
            high_start_tick = now;
        }

        if ((now - high_start_tick) >= PIR_HIGH_CONFIRM_MS)
        {
            pir_last_trigger_tick = now;
            pir_hold_until_tick = now + PIR_HOLD_MS;

            if (!pir_reported_active)
            {
                pir_reported_active = 1;
                printf("PIR,1\r\n");
				
				/*
				 * 只有 Linux 已经准备休眠/正在休眠时，
				 * PIR 才拉高 WAKE_OUT 唤醒 i.MX6ULL。
				 */
				 if (host_sleep_armed)
				 {
					 HAL_GPIO_WritePin(WAKE_OUT_GPIO_Port, WAKE_OUT_Pin, GPIO_PIN_SET);
				 }
            }
        }
    }
    else
    {
        high_confirming = 0;
    }

    /*
     * PIR 低电平后，也不要马上发 PIR,0。
     * 等保持时间结束再发。
     */
    if (pir_reported_active &&
        !pir_now &&
        (int32_t)(now - pir_hold_until_tick) >= 0)
    {
        pir_reported_active = 0;
        printf("PIR,0\r\n");
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  
  DHT11_Init();

  /* 启动 TIM2 两路 PWM */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  /* 舵机初始位置 */
  servo_h_angle = 90;
  servo_v_angle = 90;
  servo_h_target_angle = 90;
  servo_v_target_angle = 90;

  Servo_SetAngle(0, servo_h_angle);
  Servo_SetAngle(1, servo_v_angle);

  /* 开启 USART1 接收中断 */
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  printf("\r\nSTM32F103 dual servo controller start\r\n");
  printf("Send command: S,90,90\r\n");

  uint32_t led_tick = HAL_GetTick();
  
  //系统刚启动时，WAKE_OUT 必须是低电平，否则 Linux 可能刚休眠就被误唤醒。
  HAL_GPIO_WritePin(WAKE_OUT_GPIO_Port,
                  WAKE_OUT_Pin,
                  GPIO_PIN_RESET);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	PIR_Task();

	  /*
     * 1. 舵机平滑更新
     * 每隔 SERVO_SMOOTH_PERIOD_MS 调用一次。
     * 这一步必须有，否则 S 指令只会更新目标角度，舵机不会真正运动。
     */
    static uint32_t last_servo_smooth_tick = 0;

    if (HAL_GetTick() - last_servo_smooth_tick >= SERVO_SMOOTH_PERIOD_MS)
    {
        last_servo_smooth_tick = HAL_GetTick();
        Servo_SmoothUpdate();
    }

    /*
     * 2. 串口命令处理
     */
    char cmd_local_buf[64];

    if (uart_cmd_ready == 1)
    {
        __disable_irq();

        strcpy(cmd_local_buf, uart_cmd_buf);
        uart_cmd_ready = 0;

        __enable_irq();

        Process_Uart_Command(cmd_local_buf);
    }
	  
	

    // 后续可以在这里加传感器定时采集上传逻辑
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
