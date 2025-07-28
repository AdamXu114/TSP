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

#include "tsp_common_headfile.h"
#include <stdio.h>
//---------------------------------------------头文件---------------------------------------------------//

//---------------------------------------------全局变量---------------------------------------------------//
uint8_t menu_id=0;			// MenuItem ID returned by Menu_Loop()
uint8_t rx_buffer[128];
uint16_t rx_idx = 0;
uint8_t rx_flag = 0;

uint16_t route_x[999]; // 路径点X坐标数组
uint16_t route_y[999]; // 路径点Y坐标数组
uint16_t route_index = 0; // 路径点索引


uint8_t speed = 0; // 目标速度

float kp_turn_motor = 1.0f; // 原地转向电机控制的比例系数
float ki_turn_motor = 0.0f;
float kd_turn_motor = 0.0f; // 原地转向电机控制的微分系数

float kp_angle_to_err = 1.0f; // 角度转误差的比例系数
uint8_t flag_20_ms = 0; // 用于标记 20 ms 周期
uint8_t RES_value = 0; // 旋转编码器的值
uint16_t count_20ms = 0;

extern uint16_t servo1_x, servo2_y;
char show_buf[64];

//---------------------------------------------函数声明---------------------------------------------------//
void uart6_test(void); // 测试串口6


void Board_init(void){
	//DL_FlashCTL_executeClearStatus();

	//外设初始化函数
	tsp_tft18_init();
	MPU6050_Init();
	tsp_uart6_init();
    
    //中断初始化函数
	Enable_GPIOA_INT();
    ADC0_init();
	//测试函数，内部包含死循环，用于测试外设是否正常工作
    //CCD_test();
	//tsp_img_test();
	//Motor_test();
	//mpu_test();

	//tsp_tft18_test_color();
	//tsp_tft18_show_str_color(0, 0, "NUEDC-2025 SAIS@SJTU", BLUE, YELLOW);
}
//---------------------------------------------主函数---------------------------------------------------//
int main(void)
{
	uint32_t count=0;
    char buf[64];
	//系统配置初始化
	SYSCFG_DL_init();
	Board_init();
	
	while(1) {
		menu_id = tsp_menu_loop();
		switch(menu_id){
			case 0U:
				CCD_test();
				break;
			case 1U:
				tsp_img_test();	
			    break;
			case 2U:
				para_set();
				break;
            case 3U:
                uart6_test();
                break;
			case 4U:
				break;
			case 5U:
				break;
			case 6U:
				break;
			default:break;
		}
		while(S0()) {}	// wait until S3 released
	}
	// while (1) {
		
	// 	float rpy[3];
	// 	if(S0())
	// 		LED_ON();
	// 	else
	// 		LED_OFF();

	// 	if(!S2())
	// 		BUZZ_ON();
	// 	else
	// 		BUZZ_OFF();
    //     if(flag_20_ms) {
    //         flag_20_ms = 0; // 清除标志tsp_tft18_show_uint16(0, 6, count_20ms);
	// 		//tsp_tft18_show_uint16(0, 6, count_20ms);
	// 		count_20ms = 0;
    //         // MPU6050GetRPY(&rpy[0], &rpy[1], &rpy[2]);
	// 		// sprintf(buf, "Roll: %.2f", rpy[0]);
	// 		// tsp_tft18_show_str(0, 1, buf);
	// 		// sprintf(buf, "Pitch: %.2f", rpy[1]);
	// 		// tsp_tft18_show_str(0, 2, buf);
	// 		// sprintf(buf, "Yaw: %.2f", rpy[2]);
	// 		// tsp_tft18_show_str(0, 3, buf);
	// 		//Angle_Calcu();
    //     }
        
	// 	tsp_tft18_show_uint16(0, 7, RES_value);
	// }	
		
}

//---------------------------------------------函数---------------------------------------------------//
void uart6_test(void)
{
    tsp_tft18_clear(BLACK);
    uint8_t edge_step = 0; // 当前运动到哪条边
    static float t = 0.0f;    // 边上插值参数
    static float step_size = 0.1f; // 每次移动的步长，可调

    // 矩形的三个顶点
    uint16_t x1 = 0,y1 = 0;
    uint16_t x2 = 0,y2 = 0;
    uint16_t x3 = 0,y3 = 0;
    uint16_t x4 = 0,y4 = 0;

    int x_red = 0;
    int y_red = 0;
    int servo_flag = 0;
    // 处理接收到的数据
    int cx, cy;
    int tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8;
    while(!rx_flag){
        tsp_tft18_show_str(0, 0, "wait for data");
    }
    while(1){
        tsp_tft18_show_uint8(0, 2, rx_flag);
        if(rx_flag) {
            rx_flag = 0; // 清除标志
            // 解析格式:
            if(sscanf(rx_buffer, "Laser:(%d,%d)Tr:(%d,%d,%d,%d,%d,%d,%d,%d)", &cx, &cy, &tr1, &tr2, &tr3, &tr4, &tr5, &tr6, &tr7, &tr8) == 10) {
                x_red = cx; y_red = cy;
                x1 = tr1; y1 = tr2;
                x2 = tr3; y2 = tr4;
                x3 = tr5; y3 = tr6;
                x4 = tr7; y4 = tr8;
            }
        
            tsp_tft18_show_uint16(0, 4, x1);
            tsp_tft18_show_uint16(0, 5, y1);
            tsp_tft18_show_uint16(0, 6, x2);
            tsp_tft18_show_uint16(0, 7, y2);
	    }	
        if(S0())break;
    }
    while(S0()){}//等待S0释放
}
