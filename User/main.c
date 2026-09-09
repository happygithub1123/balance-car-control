#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* MPU6050 原始数据：三轴加速度和三轴陀螺仪 */
int16_t AX, AY, AZ, GX, GY, GZ;
/* 定时器中断执行超时标志和当前计数器值 */
uint8_t TimerErrorFlag;
uint16_t TimerCount;

/* 姿态角：加速度计角度、陀螺仪预测角度、互补滤波后的角度 */
float AngleAcc;
float AngleGyro;
float Angle;

/* KeyNum 为按键编号，RunFlag 为小车运行标志 */
uint8_t KeyNum, RunFlag;

/* 左右轮 PWM、平均 PWM 和转向差分 PWM，支持正负方向 */
int16_t LeftPWM, RightPWM;
int16_t AvePWM, DifPWM;

/* 角度环 PID 参数 */
PID_t AnglePID = {
	.Kp = 3,
	.Ki = 0.1,
	.Kd = 3,
	
	.OutMax = 100,
	.OutMin = -100,
};

int main(void)
{
	/* 外设初始化 */
	OLED_Init();
	MPU6050_Init();
	BlueSerial_Init();
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	Serial_Init();
	
	Timer_Init();
	
	while (1)
	{
		/* PC13 为低电平点亮 LED，用于显示小车运行状态 */
		if (RunFlag) { LED1_ON(); } else { LED1_OFF(); }
		
		/* K1 每按下一次切换一次小车运行状态 */
		KeyNum = Key_GetNum();
		if (KeyNum == 1)
		{
			if (RunFlag == 0)
			{
				/* 启动前清除 PID 上一次运行留下的误差和积分量 */
				PID_Init(&AnglePID);
				RunFlag = 1;
			}
			else
			{
				RunFlag = 0;
			}
		}
		
		/* OLED 显示 PID 参数、姿态角和陀螺仪数据 */
		OLED_Clear();
		OLED_Printf(0, 0, OLED_6X8, "  Angle");
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);
		OLED_Printf(0, 32, OLED_6X8, "T:%+05.1f", AnglePID.Target);
		OLED_Printf(0, 40, OLED_6X8, "A:%+05.1f", Angle);
		OLED_Printf(0, 48, OLED_6X8, "O:%+05.0f", AnglePID.Out);
		OLED_Printf(0, 56, OLED_6X8, "GY:%+05d", GY);
		OLED_Update();
		
		/* 处理蓝牙串口接收的数据包 */
		if (BlueSerial_RxFlag == 1)
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");
			if (strcmp(Tag, "key") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Action = strtok(NULL, ",");
				
			}
			else if (strcmp(Tag, "slider") == 0)
			{
				char *Name = strtok(NULL, ",");
				char *Value = strtok(NULL, ",");
				
				/* 通过蓝牙滑块实时修改角度环 PID 参数 */
				if (strcmp(Name, "AngleKp") == 0)
				{
					AnglePID.Kp = atof(Value);
				}
				else if (strcmp(Name, "AngleKi") == 0)
				{
					AnglePID.Ki = atof(Value);
				}
				else if (strcmp(Name, "AngleKd") == 0)
				{
					AnglePID.Kd = atof(Value);
				}
			}
			else if (strcmp(Tag, "joystick") == 0)
			{
				int8_t LH = atoi(strtok(NULL, ","));
				int8_t LV = atoi(strtok(NULL, ","));
				int8_t RH = atoi(strtok(NULL, ","));
				int8_t RV = atoi(strtok(NULL, ","));
				
				/* LV 控制目标俯仰角，RH 控制左右轮差分 PWM */
				AnglePID.Target = LV / 10.0f;
				DifPWM = RH / 2;
			}
			
			BlueSerial_RxFlag = 0;
		}
		
		BlueSerial_Printf("[plot,%f,%f]", AnglePID.Target, Angle);
	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0;
	
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		/* 每 1 ms 扫描一次按键，内部进行消抖和按键释放检测 */
		Key_Tick();
		
		Count0 ++;
		/* MPU6050 当前约 100 Hz，每 10 ms 进行一次姿态和 PID 计算 */
		if (Count0 >= 10)
		{
			Count0 = 0;
			
			/* 读取 MPU6050 六轴原始数据 */
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
			
			/* 陀螺仪零偏补偿，16 为静止时测得的 Y 轴偏置 */
			GY -= 16;
			
			/* 使用加速度计计算俯仰角 */
			AngleAcc = -atan2(AX, AZ) / 3.14159 * 180;
			
			/* 对加速度计角度进行安装方向和零点修正 */
			AngleAcc += 0.5;
			
			/* 使用陀螺仪角速度预测下一时刻角度 */
			AngleGyro = Angle + GY / 32768.0 * 2000 * 0.01;
			
			/* 互补滤波：加速度计负责长期校正，陀螺仪负责短期变化 */
			float Alpha = 0.01;
			Angle = Alpha * AngleAcc + (1 - Alpha) * AngleGyro;
			
			if (Angle > 50 || Angle < -50)
			{
				RunFlag = 0;
			}
			
			/* 小车运行时执行角度环 PID，并将输出转换为左右轮 PWM */
			if (RunFlag)
			{
				AnglePID.Actual = Angle;
				PID_Update(&AnglePID);
				AvePWM = -AnglePID.Out;
				
				/* PID 输出为平均 PWM，DifPWM 用于左右轮转向差分 */
				LeftPWM = AvePWM + DifPWM / 2;
				RightPWM = AvePWM - DifPWM / 2;
				
				/* 限制 PWM 范围，防止超过电机驱动和定时器允许值 */
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}
				
				Motor_SetPWM(1, LeftPWM);
				Motor_SetPWM(2, RightPWM);
			}
			else
			{
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}
		
		
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			TimerErrorFlag = 1;
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}
		TimerCount = TIM_GetCounter(TIM1);
	}
}
