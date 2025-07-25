#ifndef TSP_MOTOR_H
#define TSP_MOTOR_H

#include "tsp_common_headfile.h"

// 编码器初始化
void tsp_encoder_init(void);

// 获取编码器值
void tsp_encoder_get_value(uint32_t *value);

// 清零编码器
void tsp_encoder_clear(void);

#endif // TSP_MOTOR_H