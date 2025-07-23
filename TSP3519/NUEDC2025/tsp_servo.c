#include "ti_msp_dl_config.h"
#include "tsp_tft18.h"
#include "tsp_pwm.h"
#include "tsp_isr.h"
#include "tsp_servo.h"
#include <math.h>

#define SERVO_CENTER_X 1200
#define SERVO_CENTER_Y 1200
float k_angle_to_duty = 795.0f / 90.0f; // 90度对应795个脉宽单位

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

// 画圆函数
void tsp_servo_draw_circle(float radius, uint16_t steps, float distance) // radius为半径，单位cm
{
    for (uint16_t i = 0; i < steps; i++) {
        float theta = 2.0f * 3.1415926f * i / steps;
       
        float x = (float)(radius * cosf(theta));
        float y = (float)(radius * sinf(theta));
        float angle_x = tsp_length_to_angle(x, distance);
        float angle_y = tsp_length_to_angle(y, distance);

        //shabi
        // 将坐标映射到 PWM
        uint16_t servo1_pwm = tsp_angle_to_pwm(angle_x);
        uint16_t servo2_pwm = tsp_angle_to_pwm(angle_y);

        tsp_servo_angle(SERVO1, servo1_pwm);
        tsp_servo_angle(SERVO2, servo2_pwm);

        //delay_1ms(200); // 延时，保证舵机运动平滑
    }
}