/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/_intsup.h>

#include "oled.h"
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

#define CSB_MIN_D 60  // 超声波跟随状态最小距离(cm)
#define CSB_MIN_MIN_D 35 // 超声波短距离跟随状态最小距离(cm)

int i, j, k;

/* CSB CODE Start */

#define CSB_DelaY_Time 5000 //超声波中断内等待最长时间 us  
// 5000对应5ms/极限测距为85厘米  2500对应2.5ms/极限测距为42.5厘米

char CSB_MOS;                // 哪个超声波正在工作 1：A, 2：B, 3: C,
bool CSB_OK;                 // 超声波测距完成标志

unsigned int CSB_Time_ms_Start;// 记录超声波计时开始的毫秒数     

unsigned int CSB_Time_Start;// 超声波计时开始 
unsigned int CSB_Time_End;// 超声波计时开始

// unsigned int CSB_A_Time_Start;// 超声波计时开始     
// unsigned int CSB_B_Time_Start;// 超声波计时开始    
// unsigned int CSB_C_Time_Start;// 超声波计时开始

// unsigned int CSB_A_Time_End;// 超声波计时结束     
// unsigned int CSB_B_Time_End;// 超声波计时结束    
// unsigned int CSB_C_Time_End;// 超声波计时开始

unsigned int CSB_A_Time_Dat; // 超声波计时间数据
unsigned int CSB_B_Time_Dat; // 超声波计时间数据
unsigned int CSB_C_Time_Dat; // 超声波计时间数据

/* CSB CODE END */

/* Encoder CODE Start */
unsigned int Encoder_Start_Time;
unsigned int Encoder_1;     // 编码器1计数
unsigned int Encoder_2;     // 编码器2计数
unsigned int Encoder_1_Dat; // 编码器1计数,20ms刷新一次
unsigned int Encoder_2_Dat; // 编码器2计数,20ms刷新一次
/* Encoder CODE END */

unsigned int Time_1us; // 微秒计时器
unsigned int Time_1ms; // 毫秒计时器
unsigned int Time_1s; // 秒计时器

bool follow_flag = 0; // 跟随标志位 =1 时开始跟随
bool Short_follow_flagshort = 0;// 短距离跟随标志位 =1 时开始短距离跟随

/*UART CODE Start */
#define RX_BUF_SIZE 128 //串口接收缓冲区大小
uint8_t rx_buf[1];  // 中断接收的单字节缓冲区
uint8_t uart_rx_str[RX_BUF_SIZE] = {0};  // 拼接后的完整字符串
uint16_t uart_rx_len = 0;     // 当前接收的字符串长度

unsigned int Coords_1_X = 0; // 目标1X坐标
unsigned int Coords_1_Y = 0; // 目标1Y坐标
unsigned int Coords_2_X = 0; // 目标2X坐标
unsigned int Coords_2_Y = 0; // 目标2Y坐标
unsigned int Coords_3_X = 0; // 目标3X坐标
unsigned int Coords_3_Y = 0; // 目标3Y坐标

uint8_t OLED_MODS = 1;
/*UART CODE END */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 电机方向驱动，未经验证方向
void Motor(uint8_t Motor,uint8_t Motor_MOD) 
{
  if(Motor == 1)// 电机1
  {
    if(Motor_MOD == 1)// 正转
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else if(Motor_MOD == 2)// 反转
    {
      HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
  }
  else if(Motor == 2)// 电机2
  {
    if(Motor_MOD == 1)// 正转
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else if(Motor_MOD == 2)// 反转
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
  }
}

// 电机 PWM 控制，Cycle 范围 0-1000(0为0%占空比，1000为100%占空比)
void Motor_PWM(uint8_t Motor,uint8_t Cycle)
{
  if(Motor == 1)// 电机1
  {
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Cycle); // 设置占空比
  }
  else if(Motor == 2)// 电机2
  {
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Cycle); // 设置占空比
  }
}

//电机控制
void Motor_while(void) 
{

}

// 微秒延时函数
void Delay_us(uint32_t us) 
{
  uint32_t i;
  // 循环次数 = 64 * 延时微秒数（经验值，需根据实际代码效率调整）
  uint32_t loops = 64 * us;
  for (i = 0; i < loops; i++) 
  {
    __NOP(); // 空指令，消耗一个CPU周期（不同编译器可能需替换为对应空操作）
  }
}

void CSB_while(void)// 超声波测距函数 
{
  CSB_MOS = 1;
  CSB_OK = 0;
  HAL_GPIO_WritePin(CSBA_Trig_GPIO_Port, CSBA_Trig_Pin, GPIO_PIN_SET);
  Delay_us(20);
  HAL_GPIO_WritePin(CSBA_Trig_GPIO_Port, CSBA_Trig_Pin, GPIO_PIN_RESET);

  while (CSB_OK == 0 && i < (64 * 6000)) // 等待测距完成 (6ms超时保护)
    i++;
  i = 0;
  //此时已得到 CSB_A_Time_Dat

  CSB_MOS = 2;
  CSB_OK = 0;
  HAL_GPIO_WritePin(CSBB_Trig_GPIO_Port, CSBB_Trig_Pin, GPIO_PIN_SET);
  Delay_us(20);
  HAL_GPIO_WritePin(CSBB_Trig_GPIO_Port, CSBB_Trig_Pin, GPIO_PIN_RESET);

  while (CSB_OK == 0 && i < (64 * 6000)) // 等待测距完成 (6ms超时保护)
    i++;
  i = 0;
  //此时已得到 CSB_B_Time_Dat

  CSB_MOS = 3;
  CSB_OK = 0;
  HAL_GPIO_WritePin(CSBC_Trig_GPIO_Port, CSBC_Trig_Pin, GPIO_PIN_SET);
  Delay_us(20);
  HAL_GPIO_WritePin(CSBC_Trig_GPIO_Port, CSBC_Trig_Pin, GPIO_PIN_RESET);

  while (CSB_OK == 0 && i < (64 * 6000)) // 等待测距完成 (6ms超时保护)
    i++;
  i = 0;
  //此时已得到 CSB_C_Time_Dat
}

void Encoder_while(void)// 编码器数据刷新函数
{
  if(Time_1ms / 20) // 每20ms读取一次编码器数据
  {
    Encoder_1_Dat = Encoder_1;
    Encoder_2_Dat = Encoder_2;
    Encoder_1 = 0;
    Encoder_2 = 0;
  }
}

void OLED_while(void)//OLED 显示
{
  if(OLED_MODS == 1) // OLED 显示模式 1
  {
   char buffer_1[20],buffer_2[20],buffer_3[20],buffer_4[20];
   OLED_NewFrame();
   sprintf(buffer_1, "1X: %d", Coords_1_X);
   sprintf(buffer_2, "1Y: %d", Coords_1_Y);
   sprintf(buffer_3, "2X: %d", Coords_2_X);
   sprintf(buffer_4, "2Y: %d", Coords_2_Y);
   OLED_PrintString(1,1,buffer_1,&font16x16, OLED_COLOR_NORMAL);
   OLED_PrintString(60,1,buffer_2,&font16x16, OLED_COLOR_NORMAL);
   OLED_PrintString(1,30,buffer_3,&font16x16, OLED_COLOR_NORMAL);
   OLED_PrintString(60,30,buffer_4,&font16x16, OLED_COLOR_NORMAL);
   OLED_ShowFrame();
   HAL_Delay(100);
  }
  else if(OLED_MODS == 2)// OLED 显示模式 2
  {
    char buffer_5[20];
    OLED_NewFrame();
    sprintf(buffer_5, "%d", rx_buf[0]);
    OLED_PrintString(1,1,buffer_5,&font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    HAL_Delay(100);
  }
}

void UART_Parse(void)//串口数据解析
{
  if(rx_buf[0] == ']' && uart_rx_len > 0)// 接收到一帧数据
  {
    if(uart_rx_str[0] == '[')// 检查帧头
    {
      // 临时缓冲区用于处理字符串(将uart_rx_str 复制到 temp_buf)
      char temp_buf[RX_BUF_SIZE];
      strncpy(temp_buf, (char*)uart_rx_str, uart_rx_len);
      temp_buf[uart_rx_len] = '\0'; // 确保字符串结束符
    
      int count = 0;
      char *token = strtok(temp_buf + 1, "()"); // 跳过开头的'['，按()分割

      while(token != NULL && count < 3) // 最多处理3个目标
      {
        unsigned int id, x, y;
        // 解析每个目标的ID、X、Y坐标
        if(sscanf(token, "%u,%u,%u", &id, &x, &y) == 3)
        {
          switch(id)
          {
            case 1:
              Coords_1_X = x;
              Coords_1_Y = y;
              break;
            case 2:
              Coords_2_X = x;
              Coords_2_Y = y;
              break;
            case 3:
              Coords_3_X = x;
              Coords_3_Y = y;
              break;
          }
        }
        token = strtok(NULL, "()"); // 解析下一个目标
        count++;
      }
    }
    // 重置接收缓冲区
    uart_rx_len = 0;
    memset(uart_rx_str, 0, RX_BUF_SIZE);
  }
}

// 外部中断 回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
  if (GPIO_Pin == CSB_ECHO_Pin) // 进入表示ECHO变高 输出 CSB_A_Time_Dat, CSB_B_Time_Dat , CSB_C_Time_Dat
  {
    // 超声波A计时开始
    CSB_Time_Start = Time_1us;
    CSB_Time_ms_Start = Time_1ms;
    while (HAL_GPIO_ReadPin(CSB_ECHO_GPIO_Port, CSB_ECHO_Pin) == GPIO_PIN_SET)// 等待引脚变低
    {
      j++;
      if(j > (64 * CSB_DelaY_Time)) // 超时保护 5ms
      {
        break;
      }
    }
    j = 0;
    if(CSB_Time_ms_Start == Time_1ms)
    {
      CSB_Time_End = Time_1us;  // 超声波A计时结束
    }
    else if(CSB_Time_ms_Start+1 == Time_1ms)
    {
      CSB_Time_End = Time_1us + 1000;
    }

    switch(CSB_MOS)
    {
      case 1:
        CSB_A_Time_Dat = CSB_Time_End - CSB_Time_Start;
        break;
      case 2:
        CSB_B_Time_Dat = CSB_Time_End - CSB_Time_Start;
        break;
      case 3:
        CSB_C_Time_Dat = CSB_Time_End - CSB_Time_Start;
        break;
    }

    // 超声波A计时结束
    CSB_OK = 1;
    
  } 
  else if (GPIO_Pin == E1A_Pin) // 编码器A计数
  {
    Encoder_1++;
  } 
  else if (GPIO_Pin == E2A_Pin) // 编码器B计数
  {
    Encoder_2++;
  }
}

// 定时器回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) 
{
  if (htim->Instance == TIM2) // 每一微秒溢出一次
  {
    Time_1us++;
    if(Time_1us == 1000)
    {
      Time_1ms++;
      Time_1us = 0;
      if(Time_1ms == 1000)
      {
        Time_1s++;
        Time_1ms = 0;
      }
    }
  }
}

//串口回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    //HAL_UART_Transmit_IT(&huart1,rx_buf,1);// 回发接收到的字节
    if(uart_rx_len < RX_BUF_SIZE - 1) // 防止缓冲区溢出
    {
      uart_rx_str[uart_rx_len++] = rx_buf[0]; // 拼接接收到的字节
    }
    else 
    {
      uart_rx_len = 0; // 重置长度，防止溢出
    }

    UART_Parse(); // 解析接收到的数据
    
    // 重新启动中断接收（持续接收下一字节）
    HAL_UART_Receive_IT(&huart1, rx_buf, 1);
  }
  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin); // 翻转LED状态，指示收到数据
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2); // 启动，使能计时器

  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);// 启动 PWM 输出
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);// 启动 PWM 输出
  
  HAL_UART_Receive_IT(&huart1, rx_buf, 1);  // 第一次开启串口接受
  //告诉串口准备接收 1 个字节，收到后触发中断
  //此时串口进入 等待接收 状态，CPU 可执行其他任务（主循环）
  
 

  OLED_Init(); // OLED 初始化
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) 
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CSB_while(); // 超声波测距
    Encoder_while();// 编码器数据刷新函数
    OLED_while();//OLED 显示
    
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
