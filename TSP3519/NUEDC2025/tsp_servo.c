#include "ti_msp_dl_config.h"
#include "tsp_tft18.h"
#include "tsp_pwm.h"
#include "tsp_isr.h"
#include "tsp_servo.h"
#include <math.h>

#define SERVO_CENTER_X 1200
#define SERVO_CENTER_Y 1200
float k_angle_to_duty = 795.0f / 90.0f; // 90度对应795个脉宽单位

extern uint16_t route_x[999]; // 路径点X坐标数组
extern uint16_t route_y[999]; // 路径点Y坐标数组
extern uint16_t route_index = 0; // 路径点索引
extern float kp_servo; // 舵机控制的比例系数
extern float ki_servo; // 舵机控制的积分系数
extern float kd_servo; // 舵机控制的微分系数

uint16_t last_servo_x = 1200; // 上一次舵机X坐标
uint16_t last_servo_y = 1200; // 上一次舵机Y坐标
static float prev_error_x = 0.0f, prev_error_y = 0.0f;
static uint16_t integral_x = 0, integral_y = 0;
extern uint16_t point_x;
extern uint16_t point_y;// 当前激光点坐标

// 角度转脉宽函数
uint16_t tsp_angle_to_pwm(float angle)
{
    return (uint16_t)(SERVO_CENTER_X + (angle) * k_angle_to_duty);
}

// 距离转角度函数
float tsp_length_to_angle(float length, float distance)
{
    return atan2f(length, distance) * (180.0f / 3.1415926f); // atan2返回弧度，转换为角度
}


// 通过给定点坐标操控舵机
void tsp_servo_goto(uint16_t point_x, uint16_t point_y, float distance){
    float angle_x = tsp_length_to_angle(point_x, distance);
    float angle_y = tsp_length_to_angle(point_y, distance);
    // 将坐标映射到 PWM
    uint16_t servo1_pwm = tsp_angle_to_pwm(angle_x);
    uint16_t servo2_pwm = tsp_angle_to_pwm(angle_y);
    tsp_servo_angle(SERVO1, servo1_pwm);
    tsp_servo_angle(SERVO2, servo2_pwm);
}

// 画圆函数
void tsp_servo_draw_circle(float radius, uint16_t steps, float distance) // radius为半径，单位cm，steps为步数，distance为云台到屏幕的距离
{
    for (uint16_t i = 0; i < steps; i++) {
        float theta = 2.0f * 3.1415926f * i / steps;
       
        float x = (float)(radius * cosf(theta));
        float y = (float)(radius * sinf(theta));
        tsp_servo_goto(x, y, distance);

        //delay_1ms(200); // 延时，保证舵机运动平滑
    }
}

// 闭环控制云台激光笔巡线
void tsp_servo_line_follower(void){

    if (route_index >= 999) return; // 防止越界
    if(route_x[route_index] == '\0' && route_y[route_index] == '\0') return; // 如果当前点无效，直接返回

    // 目标点
    uint16_t target_x = route_x[route_index];
    uint16_t target_y = route_y[route_index];

    // 误差计算
    uint16_t error_x = target_x - point_x;
    uint16_t error_y = target_y - point_y;

    // 积分项
    integral_x += error_x;
    integral_y += error_y;

    // 微分项
    uint16_t derivative_x = error_x - prev_error_x;
    uint16_t derivative_y = error_y - prev_error_y;

    // PID输出
    float output_x = kp_servo * error_x + ki_servo * integral_x + kd_servo * derivative_x;
    float output_y = kp_servo * error_y + ki_servo * integral_y + kd_servo * derivative_y;

    // 更新历史误差
    prev_error_x = error_x;
    prev_error_y = error_y;

    // 控制舵机
    int servo_x = last_servo_x - (int)output_x;
    int servo_y = last_servo_y + (int)output_y;

    // 判断是否到达目标点
    if (fabsf(error_x) < 3.0f && fabsf(error_y) < 3.0f) {
        route_index++;
    }
}