
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
#include "math.h"


/*MPU6050测试*/
/*下载此段程序后，OLED会显示MPU6050的各项数据*/
/* MPU6050 原始数据 */
volatile int16_t AX, AY, AZ, GX, GY, GZ;

/* 姿态角：加速度计角度、陀螺仪预测角度、互补滤波后的最终角度 */
volatile float AngleAcc;
volatile float AngleGyro;
volatile float Angle;
volatile float GyroY;

/* MPU6050_GYRO_CONFIG = 0x18：量程 ±2000°/s，灵敏度 16.4 LSB/(°/s) */
#define DT              0.01f
#define RAD_TO_DEG      57.2957795f
#define GYRO_SCALE      16.4f
#define ACC_WEIGHT      0.01f

/* 陀螺仪 Y 轴零偏，需在小车静止时校准，单位为原始 ADC 值 */
volatile float GyroYOffset = -4.0f;
uint8_t TimerErrorFlag;
uint16_t TimerCount;

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	BlueSerial_Init();	//蓝牙串口初始化
	Serial_Init();		//串口初始化
	
	Timer_Init();		//定时器初始化，1ms定时中断一次
	
	while (1)
	{
		/*OLED显示*/
		OLED_Printf(0, 0, OLED_8X16, "%+06d", AX);		//显示AX
		OLED_Printf(0, 16, OLED_8X16, "%+06d", AY);		//显示AY
		OLED_Printf(0, 32, OLED_8X16, "%+06d", AZ);		//显示AZ
		OLED_Printf(64, 0, OLED_8X16, "%+06d", GX);		//显示GX
		OLED_Printf(64, 16, OLED_8X16, "%+06d", GY);	//显示GY
		OLED_Printf(64, 32, OLED_8X16, "%+06d", GZ);	//显示GZ
		OLED_Printf(0, 48, OLED_8X16, "Flag:%1d", TimerErrorFlag);	//显示TimerErrorFlag
		OLED_Printf(64, 48, OLED_8X16, "C:%05d", TimerCount);		//显示TimerCount

		//Serial_Printf("mpu:%f,%f\n",  AngleAcc, AngleGyro); // 显示MPU6050数据
		Serial_Printf("mpu:%f,%f,%f\n", AngleAcc, AngleGyro, Angle); // 显示MPU6050数据
		
		Delay_ms(20);
		
		
		/*OLED更新*/
		OLED_Update();
	}
}

void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*定时中断函数1ms自动执行一次*/

		/*进入中断函数后，立刻清标志位*/
		/*如果中断函数退出前，标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		
		/* 每 1 ms 读取一次 MPU6050 原始数据 */
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

		/*
		 * SMPLRT_DIV = 9 时，MPU6050 输出数据约为 100 Hz，
		 * 即约每 10 ms 更新一次，因此每 10 次中断进行一次姿态融合。
		 */
		static uint8_t FusionCount = 0;
		FusionCount++;
		if (FusionCount >= 10)
		{
			FusionCount = 0;

			/* 1. 加速度计计算俯仰角 */
			AngleAcc = -atan2f((float)AX, (float)AZ) * RAD_TO_DEG;

			/* 2. 陀螺仪原始值转换为 °/s，并减去 Y 轴零偏 */
			GyroY = ((float)GY - GyroYOffset) / GYRO_SCALE;

			/* 3. 从上一次滤波后的角度进行陀螺仪预测 */
			AngleGyro = Angle + GyroY * DT;

			/*
			 * 4. 互补滤波：加速度计负责长期校正，
			 *    陀螺仪负责短期快速变化。
			 */
			Angle = ACC_WEIGHT * AngleAcc
			      + (1.0f - ACC_WEIGHT) * AngleGyro;
		}
		
		/*中断函数退出前，再次检查标志位*/
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			/*标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
			/*置TimerErrorFlag为1，表示定时中断错误*/
			TimerErrorFlag = 1;

			/*清标志位，避免中断连续触发，导致主函数完全无法执行*/
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}

		/*中断函数退出前，读取计数器的值，此值可用于测量中断函数的具体执行时间*/
		TimerCount = TIM_GetCounter(TIM1);
	}
}
