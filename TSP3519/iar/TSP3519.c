/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "tsp_isr.h"
#include "tsp_gpio.h"
#include "tsp_i2c.h"
#include "TSP_MPU6050.h"
#include "TSP_TFT18.h"
#include "tsp_pwm.h"
uint8_t rx_buffer[128];
uint16_t rx_idx = 0;
uint8_t rx_flag = 0;

float kp_motor = 1.0f; // 电机控制的比例系数
float ki_motor = 0.0f;
float kd_motor = 0.0f; // 电机控制的微分系数
float kp_servo = 0.0f; // 舵机控制的比例系数
float ki_servo = 0.0f;
float kd_servo = 0.0f; // 舵机控制的微分系数

float kp_turn_motor = 1.0f; // 原地转向电机控制的比例系数
float ki_turn_motor = 0.0f;
float kd_turn_motor = 0.0f; // 原地转向电机控制的微分系数

float kp_angle_to_err = 1.0f; // 角度转误差的比例系数
uint8_t flag_20_ms = 0; // 用于标记 20 ms 周期
uint32_t dt = 200;

int16_t gz = 0; // 陀螺仪 Z 轴数据
int main(void)
{
	uint32_t count=0;
	
	SYSCFG_DL_init();
	DL_TimerG_startCounter(Servo_INST);
	DL_TimerA_startCounter(Motor_INST);
	//DL_FlashCTL_executeClearStatus();
	
	tsp_tft18_init();
	//tsp_tft18_test_color();
	tsp_tft18_show_str_color(0, 0, "NUEDC-2025 SAIS@SJTU", BLUE, YELLOW);
    MPU6050_Init();
    
	while (1) {
		float rpy[3];
		if(S0())
			LED_ON();
		else
			LED_OFF();

		if(!S2())
			BUZZ_ON();
		else
			BUZZ_OFF();
		
		float tmp = 0.0f;
        if(flag_20_ms) {
            flag_20_ms = 0; // 清除标志
            MPU6050GetRPY(&rpy[0], &rpy[1], &rpy[2]);
        }
        
		char buf[64];
        sprintf(buf, "Roll: %.2f", rpy[0]);
        tsp_tft18_show_str(0, 1, buf);
        sprintf(buf, "Pitch: %.2f", rpy[1]);
        tsp_tft18_show_str(0, 2, buf);
        sprintf(buf, "Yaw: %.2f", rpy[2]);
        tsp_tft18_show_str(0, 3, buf);


        //tsp_tft18_show_uint16(0, 5, count++);
	}	
			  
}
