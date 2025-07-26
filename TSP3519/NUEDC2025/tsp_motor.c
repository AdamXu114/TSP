#include "tsp_motor.h"

// 电机速度相关变量
extern int16_t target_speed_qei1; // 目标速度 QEI1
extern int16_t target_speed_qei2; // 目标速度 QEI2
extern int16_t current_speed_qei1; // 当前速度 QEI1
extern int16_t current_speed_qei2; // 当前速度 QEI2

// 电机PID参数
extern float kp_motor;
extern float ki_motor;
extern float kd_motor;
#define MOTOR_DC_LIMIT	4000

// 舵机与转向PID参数
extern float kp_servo;
extern float ki_servo;
extern float kd_servo;
extern float kp_turn_motor;
extern float ki_turn_motor;
extern float kd_turn_motor;

// 角度误差参数与姿态
extern float kp_angle_to_err;
extern float yaw;
extern float roll;
extern float pitch;


void tsp_encoder_init(void)
{
    TSP_QEI_Init();
}

void tsp_encoder_get_value(uint32_t *value){
    value[0] = TSP_QEI1_GetCount();
    value[1] = TSP_QEI2_GetCount();
    //tsp_tft18_show_str_color(0, 6, "Encoder Value:", BLUE, YELLOW);
}

void tsp_encoder_clear(void){
    TSP_QEI1_ResetCount();
    TSP_QEI2_ResetCount();
}

// 快捷测试电机和编码器功能
void Motor_test(void){
    while(1){
        SLEEP_HIGH();
        tsp_motor_control(50,MOTOR2);
        uint32_t value[2];
        tsp_encoder_get_value(value);
        tsp_encoder_clear();
        // 这里可以添加更多的测试逻辑
        // 例如打印编码器值等
        char buf[40];
        tsp_tft18_show_uint16(0, 4, value[0]);
        tsp_tft18_show_uint16(0, 5, value[1]);
    }
}

// 电机速度闭环控制
void tsp_speed_close_loop(){
    // 二阶增量式PID
    static float output1 = 0, output2 = 0;
    static int16_t e1_k = 0, e1_k_1 = 0, e1_k_2 = 0;
    static int16_t e2_k = 0, e2_k_1 = 0, e2_k_2 = 0;

    float delta_out1, delta_out2;

    // 当前误差
    e1_k = target_speed_qei1 - current_speed_qei1;
    e2_k = target_speed_qei2 - current_speed_qei2;

    // 增量式PID公式
    delta_out1 = kp_motor * (e1_k - e1_k_1)
               + ki_motor * e1_k
               + kd_motor * (e1_k - 2 * e1_k_1 + e1_k_2);
    delta_out2 = kp_motor * (e2_k - e2_k_1)
               + ki_motor * e2_k
               + kd_motor * (e2_k - 2 * e2_k_1 + e2_k_2);

    output1 += delta_out1;
    output2 += delta_out2;

    // 限幅
    if (output1 > 4000) output1 = 4000;
    if (output1 < -4000) output1 = -4000;
    if (output2 > 4000) output2 = 4000;
    if (output2 < -4000) output2 = -4000;

    tsp_motor_control((int16_t)output1, MOTOR1);
    tsp_motor_control((int16_t)output2, MOTOR2);

    // 更新误差历史
    e1_k_2 = e1_k_1;
    e1_k_1 = e1_k;
    e2_k_2 = e2_k_1;
    e2_k_1 = e2_k;
}


//停车函数
void tsp_motor_stop(void)
{
	tsp_motor_control(0, MOTOR1);
	tsp_motor_control(0, MOTOR2);
	tsp_tft18_show_str_color(0, 1, "Motor Stop", BLUE, YELLOW);
}

// 实现原地左转或右转一定角度的函数
void tsp_motor_turn_inplace(uint8_t dir, uint16_t duty_cycle_limit, uint16_t angle) 
{
	uint16_t dc;

	dc = duty_cycle_limit;
	if (MOTOR_DC_LIMIT < duty_cycle_limit)
		dc = MOTOR_DC_LIMIT;
	float target_yaw = yaw;
	switch(dir){
		case LEFT:
		target_yaw = yaw + angle; // yaw是当前角度
		break;
		case RIGHT:
		target_yaw = yaw - angle; // yaw是当前角度
		break;
		default:
			return; // 无效方向
	}
	
	if (target_yaw > 360.0f) target_yaw -= 360.0f;
	if (target_yaw < 0.0f) target_yaw += 360.0f;

	tsp_tft18_show_str_color(0, 1, "Motor Turn Left", BLUE, YELLOW);

	// 简单PID参数
	float error, output;
	while (1) {
		// 计算误差
		error = target_yaw - yaw;
		// 保证误差在[-180,180]区间
		if (error > 180.0f) error -= 360.0f;
		if (error < -180.0f) error += 360.0f;

		// 到达目标角度则退出
		if (fabsf(error) < 2.0f) break;

		// 简单P控制
		output = kp_turn_motor * error;
		if (output > dc) output = dc;
		if (output < 10) output = 10; // 最小转速
		if (dir == LEFT) {
			tsp_motor_control((int16_t)output, MOTOR1); // 左转
			tsp_motor_control(-(int16_t)output, MOTOR2); // 右转
		} else if (dir == RIGHT) {
			tsp_motor_control(-(int16_t)output, MOTOR1); // 右转
			tsp_motor_control((int16_t)output, MOTOR2); // 左转
		}
		MPU6050GetRPY(&roll, &pitch, &yaw); // 获取当前的roll、pitch、yaw角度
	}
}

// // PID控制电机速度
// void tsp_motor_speed_pid(uint16_t target_speed_pid, uint8_t motor)
// {
// 	static float integral = 0.0f;
// 	static float prev_error = 0.0f;
// 	float error = target_speed_pid - current_speed;
// 	float output;

// 	integral += error;
// 	float derivative = error - prev_error;

// 	output = kp_motor * error + ki_motor * integral + kd_motor * derivative;

// 	// 限制输出范围
// 	if (output > MOTOR_DC_LIMIT) output = MOTOR_DC_LIMIT;
// 	if (output < 0) output = 0;

// 	// 设置PWM占空比
// 	if (motor == MOTOR1)
// 		DL_TimerG_setCaptureCompareValue(Motor_INST, (uint16_t)output, DL_TIMER_CC_0_INDEX);
// 	else if (motor == MOTOR2)
// 		DL_TimerG_setCaptureCompareValue(Motor_INST, (uint16_t)output, DL_TIMER_CC_1_INDEX);
// 	else
// 		return; // 无效电机编号

// 	prev_error = error;
// }

// 差速底盘巡线
void tsp_line_follower(float err){
	target_speed_qei1 += kp_angle_to_err * err; // 根据误差调整目标速度
	target_speed_qei2 -= kp_angle_to_err * err; // 反向调整另一个电机的速度
	// 在主循环的其他地方应调用tsp_speed_close_loop()来执行速度闭环控制
}