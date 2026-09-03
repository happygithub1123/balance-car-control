
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"


uint8_t keynum = 0;		//按键编号
int8_t PWML, PWMR;		//左、右电机PWM占空比

volatile float SpeedL, SpeedR;		//左、右轮速度

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	LED_Init();
	Key_Init();
	Motor_Init();
	Encoder_Init();
	
	Timer_Init();		//定时器初始化，1ms定时中断一次

	
	
	while (1)
	{
		/*按键*/
		keynum = Key_GetNum();		//按键扫描
		if (keynum == 1)
		{
			PWML += 10;		//按键1按下，左电机PWM占空比设为100
		}
		else if (keynum == 2)
		{
			PWML -= 10;		//按键2按下，左电机PWM占空比减10
		}
		else if (keynum == 3)
		{
			PWMR += 10;		//按键3按下，右电机PWM占空比加10
		}
		else if (keynum == 4)
		{
			PWMR -= 10;		//按键4按下，右电机PWM占空比减10
		}
		if (PWML > 100) PWML = 100;	//限制左电机PWM占空比最大为100
		if (PWML < -100) PWML = -100;		//限制左电机PWM占空比最小为-100
		if (PWMR > 100) PWMR = 100;	//限制右电机PWM占空比最大为100
		if (PWMR < -100) PWMR = -100;		//限制右电机PWM占空比最小为-100

		Motor_SetSpeed(1, PWML); 
		Motor_SetSpeed(2, PWMR);

		/*OLED显示*/
		OLED_Printf(0, 0, OLED_8X16, "PWML:%+04d", PWML);		//显示左轮的PWM
		OLED_Printf(0, 16, OLED_8X16, "PWMR:%+04d", PWMR);		//显示右轮的PWM
		OLED_Printf(0, 32, OLED_8X16, "SpdL:%+06.2f", SpeedL);	//显示左轮的速度
		OLED_Printf(0, 48, OLED_8X16, "SpdR:%+06.2f", SpeedR);	//显示右轮的速度
		
		/*OLED更新*/
		OLED_Update();
	}
}





//定时器1中断服务函数，每1ms进入一次
void TIM1_UP_IRQHandler(void)
{
    static uint16_t Count = 0;

    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        /* 按键仍然每1ms扫描一次 */
        Key_Tick();

        /* 累计50次，即每50ms计算一次速度 */
        Count++;

        if (Count >= 50)
        {
            Count = 0;

            SpeedL = Encoder_Get(1) / 44.0 / 0.05 / 9.27666;	//左轮速度，单位为r/s
            SpeedR = Encoder_Get(2) / 44.0 / 0.05 / 9.27666;	//右轮速度，单位为r/s
        }

        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
