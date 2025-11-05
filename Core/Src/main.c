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
 未完善的部分 电机转动方向 
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "font.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/_intsup.h>

#include "oled.h"
#include "NRF24L01.h"
#include <inttypes.h>// 豆包来的PRId32 等宏定义
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

void CSB_while_Delay(void);// 超声波测距延时函数

#define CSB_MIN_Dat 60  // 超声波跟随状态最小距离(cm)
#define CSB_MIN_MIN_Dat 35 // 超声波短距离跟随状态最小距离(cm)


// 电机 PID 参数
#define Motor_KP 3
#define Motor_KI 0
#define Motor_KD 0

//K230跟随模式参数---------------------------------------------------------
#define screenCenterX 320//屏幕中心x 坐标
#define screenCenterY 240//屏幕中心y 坐标
#define maxUnresponsiveCoordDiff 80//最大无响应坐标差值
#define targetSpeedValue 200 // 目标速度值
#define turnSpeedDiffCoeff 1 // 转弯时速度差值系数

//#define slowdownValueWhenNoPersonDetected 100 // 未检测到人时减速值
#define circlingSpeed 100 // 原地转圈速度差值


//无线遥控模式参数---------------------------------------------------------
#define N_targetSpeedValue 200 // 目标速度值
#define N_turnSpeedDiff 75 // 转弯时速度差



uint8_t NRF_MODS = 0;//无线遥控模式 0 停止,1遥控跟随,2摄像头跟随
unsigned int Joystick__XDat = 0;
unsigned int Joystick__YDat = 0;

/* CSB CODE Start */

char CSB_MOS;                // 哪个超声波正在工作 1：A, 2：B, 3: C,

unsigned int CSB_Time_ms_Start;// 超声波计时开始 
unsigned int CSB_Time_ms_End;// 超声波计时开始

unsigned int CSB_A_Time_Dat; // 超声波计时间数据
unsigned int CSB_B_Time_Dat; // 超声波计时间数据
unsigned int CSB_C_Time_Dat; // 超声波计时间数据

unsigned int CSB_A_Dis_Dat; // 超声波距离数据 单位 cm

/* CSB CODE END */

/* Encoder CODE Start */
unsigned int Encoder_Start_Time;
unsigned int Encoder_1;     // 编码器1计数
unsigned int Encoder_2;     // 编码器2计数
unsigned int Encoder_1_Dat; // 编码器1计数,20ms刷新一次
unsigned int Encoder_2_Dat; // 编码器2计数,20ms刷新一次
/* Encoder CODE END */

unsigned int Time_1ms; // 毫秒计时器
uint32_t Time_1s; // 秒计时器 溢出需要 1193046.47 小时

unsigned int Delay_Time_ms_Start;//延时计算开始时间
unsigned int Delay_Time_s_Start;//延时计算开始时间
unsigned int Delay_Time_ms_End;//延时计算结束时间

unsigned int Delay_Time_ms_Start;//延时计算开始时间
unsigned int Delay_Time_s_Start;//延时计算开始时间

unsigned int Tim2_Time_Start;//开始时间
unsigned int Tim2_Time_End;//结束时间

bool follow_flag = 0; // 跟随标志位 =1 时开始跟随
bool Short_follow_flagshort = 0;// 短距离跟随标志位 =1 时开始短距离跟随
bool neverDetectedHuman_flag = 1; // 从未检测到人 标志位
unsigned int notRecognized_Start_Time_1s = 0; // 未识别到人计时开始时间


uint8_t	Receive[17]; //NRF24L01接收的内容
uint16_t NRF24L01_Dat; //NRF24L01接收的数据（直接转换为整数）

//UART参数---------------------------------------------------------
#define RX_BUF_SIZE 128 //串口接收缓冲区大小
uint8_t rx_buf[1];  // 中断接收的单字节缓冲区
uint8_t uart_rx_str[RX_BUF_SIZE] = {0};  // 拼接后的完整字符串
uint16_t uart_rx_len = 0;     // 当前接收的字符串长度


bool person = 0;// 是否检测到人

uint16_t person_NODat = 0;// 是否检测到人累积次数
#define person_NODat_MAX 20 //检测不到人的退出次数

uint16_t PWM_1,PWM_2;//PWM 监测值

unsigned int Coords_1_X = 0; // 目标1X坐标
unsigned int Coords_1_Y = 0; // 目标1Y坐标
unsigned int Coords_2_X = 0; // 目标2X坐标
unsigned int Coords_2_Y = 0; // 目标2Y坐标
unsigned int Coords_3_X = 0; // 目标3X坐标
unsigned int Coords_3_Y = 0; // 目标3Y坐标





/*UART CODE END */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 电机方向驱动，未验证方向
//Motor : 电机编号 1 或 2
//Motor_MOD : 电机转动模式 1 正转 2 反转
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
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    else if(Motor_MOD == 2)// 反转
    {
      HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
  }
}

// 电机 PWM 控制，Cycle 范围 0-1000(0为0%占空比，1000为100%占空比)
//Motor : 电机编号 1 或 2
//Cycle : 占空比 0-1000
void Motor_PWM(uint8_t Motor,int16_t Cycle)
{
  if(Cycle > 1000)
  {
    Cycle = 1000; // 限制最大值为1000
  }
  if(Cycle < 0)
  {
    Cycle = 0; // 限制最小值为0
  }
  if(Motor == 1)// 电机1
  {
    PWM_1 = Cycle;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Cycle); // 设置占空比
  }
  else if(Motor == 2)// 电机2
  {
    PWM_2 = Cycle;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Cycle); // 设置占空比
  }
}

// 电机 PID 控制 返回占空比值
//Target_value : 目标值
//Actual_value : 实际值
uint16_t Motor_PID(uint16_t Target_value,uint16_t Actual_value)
{
  int16_t Error;
  Error = Target_value - Actual_value;
  if(Error < 0)
  {
    Error = 0;
  }
  return Error * Motor_KP; // 简单比例控制
}

//摄像头跟随 包含了跟随到人和没跟随到人两种情况
void Motor_K230_follow(void)
{
  Motor(1,1); // 电机1正转
  Motor(2,1); // 电机2正转
  if(person == 0)//如果没有识别到人 停止
  {
    Motor_PWM(1,0); // 电机1 占空比
    Motor_PWM(2,0); // 电机2 占空比
    return;
  }
  if(CSB_A_Dis_Dat < 15)//距离过近
  {
    Motor_PWM(1,0); // 电机1 占空比
    Motor_PWM(2,0); // 电机2 占空比
    return;
  }
  if(screenCenterX < Coords_1_X )//在右侧
  {
    if((Coords_1_X - screenCenterX) < maxUnresponsiveCoordDiff)//在中心区域
    {
      Motor_PWM(1,Motor_PID(targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
      Motor_PWM(2,Motor_PID(targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
      return;
    }
  }
  else if(Coords_1_X < screenCenterX)//在左侧
  {
    if((screenCenterX - Coords_1_X) < maxUnresponsiveCoordDiff)//在中心区域
    {
      Motor_PWM(1,Motor_PID(targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
      Motor_PWM(2,Motor_PID(targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
      return;
    }
  }
  
  
  if(Coords_1_X < screenCenterX)// 目标在左侧 (screenCenterX-Coords_1_X)误差值,人离镜头中心越远值越大
  {
    //差值计算公式 = 目标速度-(误差值*系数)
    Motor_PWM(1,Motor_PID(targetSpeedValue - (screenCenterX - Coords_1_X)*turnSpeedDiffCoeff + maxUnresponsiveCoordDiff,Encoder_1_Dat)); // 电机1 占空比
    Motor_PWM(2,Motor_PID(targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
  }
  else if(screenCenterX < Coords_1_X)// 目标在右侧
  {
    Motor_PWM(1,Motor_PID(targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
    Motor_PWM(2,Motor_PID(targetSpeedValue - (Coords_1_X - screenCenterX)*turnSpeedDiffCoeff + maxUnresponsiveCoordDiff,Encoder_2_Dat)); // 电机2 占空比
  }
  
}

//无线遥控跟随
void Motor_NRF24L01_follow(void)
{
  //Joystick__XDat,Joystick__YDat  0 到 4095
  uint8_t Xmod,Ymod;
  if(Joystick__XDat > 4000)
  {
    Xmod = 2;//左
  }
  else if(Joystick__XDat < 20)
  {
    Xmod = 1;//右
  }
  else {
  {
    Xmod = 0;//中
  }
  }
  if(Joystick__YDat > 4000)
  {
    Ymod = 1;//下
  }
  else if(Joystick__YDat < 20)
  {
    Ymod = 2;//上
  }
  else {
  {
    Ymod = 0;//中
  }
  }
  if(Ymod == 0 && Xmod == 0)// 停止
  {
    Motor_PWM(1,0); // 电机1 占空比
    Motor_PWM(2,0); // 电机2 占空比
    return;
  }
  switch(Ymod)//设置了方向
  {
    case 2://前进
      Motor(1,1); // 电机1正转
      Motor(2,1); // 电机2正转
    break;
    case 1://后退
      Motor(1,2); // 电机1反转
      Motor(2,2); // 电机2反转
    break;
  }
  switch(Xmod)//设置了速度
  {
    case 1://左转
      if(Ymod == 0)// 停止状态下左转
      {
        Motor(1,2); // 电机1反转
        Motor(2,1); // 电机2正转
        Motor_PWM(1,Motor_PID(N_targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
        Motor_PWM(2,Motor_PID(N_targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
        return;
      }
      Motor_PWM(1,Motor_PID(N_targetSpeedValue-N_turnSpeedDiff,Encoder_1_Dat)); // 电机1 占空比
      Motor_PWM(2,Motor_PID(N_targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
    break;
    case 2://右转
      if(Ymod == 0)// 停止状态下右转
      {
        Motor(1,1); // 电机1正转
        Motor(2,2); // 电机2反转
        Motor_PWM(1,Motor_PID(N_targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
        Motor_PWM(2,Motor_PID(N_targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
        return;
      }
      Motor_PWM(1,Motor_PID(N_targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
      Motor_PWM(2,Motor_PID(N_targetSpeedValue-N_turnSpeedDiff,Encoder_2_Dat)); // 电机2 占空比
    break;
    default://直行
      Motor_PWM(1,Motor_PID(N_targetSpeedValue,Encoder_1_Dat)); // 电机1 占空比
      Motor_PWM(2,Motor_PID(N_targetSpeedValue,Encoder_2_Dat)); // 电机2 占空比
    break;
  }
}
//原地转圈
void Motor_turnInPlace(void)
{
  if(screenCenterX - Coords_1_X < maxUnresponsiveCoordDiff || 
     Coords_1_X - screenCenterX < maxUnresponsiveCoordDiff)// 目标X坐标在屏幕中心附近
  {
    return; //不转圈
  }
  else if(screenCenterX > Coords_1_X)// 目标在右侧
  {
    Motor(1,1); // 电机1正转
    Motor(2,2); // 电机2反转
  }
  else if(screenCenterX < Coords_1_X)// 目标在左侧
  {
    Motor(1,2); // 电机1反转
    Motor(2,1); // 电机2正转
  }
  Motor_PWM(1,circlingSpeed - circlingSpeed); // 电机1 占空比
  Motor_PWM(2,circlingSpeed - circlingSpeed); // 电机2 占空比
}

//电机控制
void Motor_while(void) 
{
  switch(NRF_MODS)
  {
    case 0 :
      Motor_PWM(1,0); // 电机1 占空比
      Motor_PWM(2,0); // 电机2 占空比
    break;
    case 1 ://摄像头跟随
      Motor_K230_follow(); // 执行摄像头跟随
    break;
    case 2 ://无线遥控
      Motor_NRF24L01_follow(); // 执行无线遥控跟随
    break;
  }
}

// 微秒延时函数
void Delay_us(uint32_t us) 
{
  uint32_t i;
  // 循环次数 = 72 * 延时微秒数（经验值，需根据实际代码效率调整）
  uint32_t loops = 72 * us;
  for (i = 0; i < loops; i++) 
  {
    __NOP(); // 空指令，消耗一个CPU周期（不同编译器可能需替换为对应空操作）
  }
}

// 超声波测距函数 
void CSB_while(void)
{
  CSB_MOS = 1;
  HAL_GPIO_WritePin(CSBA_Trig_GPIO_Port, CSBA_Trig_Pin, GPIO_PIN_SET);
  Delay_us(20);
  HAL_GPIO_WritePin(CSBA_Trig_GPIO_Port, CSBA_Trig_Pin, GPIO_PIN_RESET);
}

// 编码器数据刷新函数
void Encoder_while(void)
{
  if(Time_1ms % 20 == 0) // 每20ms读取一次编码器数据
  {
    Encoder_1_Dat = Encoder_1;
    Encoder_2_Dat = Encoder_2;
    Encoder_1 = 0;
    Encoder_2 = 0;
  }
}

//OLED 显示
void OLED_while(void)
{

  char buffer_1[48],buffer_2[30],buffer_3[30],buffer_4[30],buffer_5[30],buffer_6[30];
  int OLED_MODS = 5;
  if(OLED_MODS == 1) // OLED 显示坐标
  {
   OLED_NewFrame();
   sprintf(buffer_1, "1X: %d", Coords_1_X);
   sprintf(buffer_2, "1Y: %d", Coords_1_Y);
   OLED_PrintString(1,1,buffer_1,&font16x16, OLED_COLOR_NORMAL);
   OLED_PrintString(60,1,buffer_2,&font16x16, OLED_COLOR_NORMAL);
   OLED_ShowFrame();
  }
  else if(OLED_MODS == 2)// OLED 显示串口
  {
    char buffer_5[20];
    OLED_NewFrame();
    sprintf(buffer_5, "%d", rx_buf[0]);
    OLED_PrintString(1,1,buffer_5,&font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }
  else if(OLED_MODS == 3)// OLED 显示超声波数据超声波
  {
    OLED_NewFrame();
    sprintf(buffer_1, "A: %d", CSB_A_Time_Dat);
    sprintf(buffer_2, "B: %d", CSB_B_Time_Dat);
    sprintf(buffer_3, "C: %d", CSB_C_Time_Dat);
    sprintf(buffer_4, "CSB_MOS: %d", CSB_MOS);
    sprintf(buffer_5, "ms_Start: %d", CSB_Time_ms_Start);
    sprintf(buffer_6, "ms_End: %d",CSB_Time_ms_End);
    OLED_PrintString(1,1,buffer_1,&font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(1,20,buffer_2,&font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(1,40,buffer_3,&font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(40,1,buffer_4,&font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(40,20,buffer_5,&font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(40,40,buffer_6,&font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }
  else if(OLED_MODS == 4)// OLED 显示
  {
    OLED_NewFrame();
    sprintf(buffer_4, "A: %d", CSB_A_Time_Dat);
    sprintf(buffer_5, "Time_End: %d", Tim2_Time_End);
    sprintf(buffer_6, "ms_End: %d",CSB_Time_ms_End);
    //OLED_PrintString(1,1,buffer_4,&font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(1,20,buffer_5,&font16x16, OLED_COLOR_NORMAL);
    //OLED_PrintString(1,40,buffer_6,&font16x16, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
    
  }
  else if(OLED_MODS == 5)// OLED 显示
  {
    OLED_NewFrame();
    sprintf(buffer_1, "K230 X:%d Y:%d", Coords_1_X, Coords_1_Y);
    sprintf(buffer_2, "cm:%d E1:%d E2:%d",CSB_A_Dis_Dat,Encoder_1_Dat,Encoder_2_Dat);
    sprintf(buffer_3, "NRF:%d,X:%d,Y:%d",NRF_MODS,Joystick__XDat,Joystick__YDat);
    sprintf(buffer_4, "PWM1:%d,PWM2:%d",PWM_1,PWM_2);
    OLED_PrintASCIIString(1,1,buffer_1,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,10,buffer_2,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,20,buffer_3,&afont8x6, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(1,30,buffer_4,&afont8x6, OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }
}

//串口数据解析
void UART_Parse(void)
{
  if(rx_buf[0] == 'n')
  {
    person = 0; // 未检测到人
  }
  
  if(rx_buf[0] == ']' && uart_rx_len > 0)// 接收到一帧数据
  {
    person = 1; // 检测到人
    person_NODat = 0; //清空检测次数
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

// NRF24L01数据解析
void ParseNRFData_while(void) 
{
    // 查找第一个逗号分隔符
    char* comma1 = strchr((char*)Receive, ',');
    if (comma1 != NULL) 
    {
        // 提取mod值
        *comma1 = '\0';  // 截断字符串
        NRF_MODS = (uint8_t)strtol((char*)Receive, NULL, 10);
        
        // 查找第二个逗号分隔符
        char* comma2 = strchr(comma1 + 1, ',');
        if (comma2 != NULL) 
        {
            // 提取X值
            *comma2 = '\0';  // 截断字符串
            Joystick__XDat = (uint16_t)strtol(comma1 + 1, NULL, 10);
            
            // 提取Y值
            Joystick__YDat = (uint16_t)strtol(comma2 + 1, NULL, 10);
        }
    }
}

void NRF24L01_Z_Init(void)
{
  // while(NRF24L01_Check())
  // {
  //     // printf("硬件查寻不到NRF24L01无线模块,请检查接线是否错误\r\n"); 
  //     HAL_Delay(1000);
  // }
	NRF24L01_RX_Mode();//设置为接受模式
}

void NRF24L01_while(void)
{
  if(NRF24L01_RxPacket(Receive)==0)
    {
      Receive[16]=0;//加入字符串结束符   
      NRF24L01_Dat = atoi((char*)Receive); // 将接收到的数据转换为整数
    }
}

// 外部中断 回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
  if (GPIO_Pin == CSB_ECHO_1_Pin) // 进入表示ECHO变高 
  {
    HAL_TIM_Base_Start(&htim2); // 启动，使能计时器
  } 
  else if(GPIO_Pin == CSB_ECHO_2_Pin)//进入表示ECHO变低 
  {
    HAL_TIM_Base_Stop(&htim2); // 停止，禁止计时器
    Tim2_Time_End = __HAL_TIM_GET_COUNTER(&htim2); // 读取计时器的计数值 单位是100us
    CSB_A_Dis_Dat = Tim2_Time_End * 0.7; // 计算距离 cm
     
    __HAL_TIM_SET_COUNTER(&htim2,0); // 清零计数器，为下一次测量做准备
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
  if (htim->Instance == TIM2) // 每一毫秒溢出一次
  {
    Time_1ms++;
    if(Time_1ms >= 1000)
    {
      Time_1s++;
      Time_1ms = 0;
    }
  }
}

//串口回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
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
}

// 测试函数
void test(void)
{
  Motor(1,1); // 电机1 
  Motor(2,1); // 电机2
  Motor_PWM(1,500); // 电机1 占空比
  Motor_PWM(2,500); // 电机2 占空比
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

  HAL_Init();

  /*USER CODE BEGIN Init*/

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);// 启动 PWM 输出
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);// 启动 PWM 输出
  
  HAL_UART_Receive_IT(&huart1, rx_buf, 1);  // 第一次开启串口接受
  //告诉串口准备接收 1 个字节，收到后触发中断
  //此时串口进入 等待接收 状态，CPU 可执行其他任务（主循环）
  
 

  OLED_Init(); // OLED 初始化

  NRF24L01_Z_Init();//NRF24L01 初始化

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) 
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CSB_while(); // 超声波测距
    Encoder_while();// 编码器数据刷新函数
    NRF24L01_while();//NRF24L01 数据接收
    ParseNRFData_while();//NRF24L01 数据解析
    OLED_while();//OLED 显示
    //test();// 测试函数
    Motor_while();// 电机控制
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
