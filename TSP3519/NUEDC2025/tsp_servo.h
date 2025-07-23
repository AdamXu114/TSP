#ifndef TSP_SERVO_H
#define TSP_SERVO_H

#include <stdint.h>

// 角度转脉宽
uint16_t tsp_angle_to_pwm(float angle);

// 距离转角度
float tsp_length_to_angle(float length, float distance);

// 画圆函数
void tsp_servo_draw_circle(float radius, uint16_t steps, float distance);

#endif // TSP_SERVO_H
