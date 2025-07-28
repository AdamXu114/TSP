#include "tsp_pid.h"



float kp_motor = 1.0f; // 电机控制的比例系数
float ki_motor = 0.0f;
float kd_motor = 0.0f; // 电机控制的微分系数
float kp_servo = 0.2f; // 舵机控制的比例系数
float ki_servo = 0.0f;
float kd_servo = 0.0f; // 舵机控制的微分系数

extern uint16_t servo1_x ;
extern uint16_t servo2_y;
extern uint16_t last_servo_x; // 上一次舵机X坐标
extern uint16_t last_servo_y; // 上一次舵机Y坐标
// PID控制电机速度
void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor)
{
	static float integral = 0.0f;
	static float prev_error = 0.0f;
    float current_speed = Get_speed(motor);
	float error = target_speed_pid - current_speed;
	float output;

	integral += error;
	float derivative = error - prev_error;

	output = kp_motor * error + ki_motor * integral + kd_motor * derivative;

	// 限制输出范围
	if (output > MOTOR_DC_LIMIT) output = MOTOR_DC_LIMIT;
	if (output < 0) output = 0;

	//tsp_motor_control((int16_t)output, motor);

	prev_error = error;
}


void tsp_servo_control_pid(float target_x, float target_y, float current_x, float current_y){
    static float prev_error_x = 0.0f, prev_error_y = 0.0f;
    static uint16_t integral_x = 0, integral_y = 0;
    // TODO: 实现舵机控制PID算法
    // 根据目标点坐标和当前点坐标计算误差
    float error_x = target_x - current_x;
    float error_y = target_y - current_y;

    // 积分项及限幅
    integral_x += error_x;
    integral_y += error_y;
    if(integral_x > 1000) integral_x = 1000;
    if(integral_y > 1000) integral_y = 1000;
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
    servo1_x = last_servo_x - (int)output_x;
    servo2_y = last_servo_y + (int)output_y;
    if (servo1_x < 500) servo1_x = 500;
    if (servo1_x > 2050) servo1_x = 2050;
    if (servo2_y < 500) servo2_y = 500;
    if (servo2_y > 2050) servo2_y = 2050;
    
}


