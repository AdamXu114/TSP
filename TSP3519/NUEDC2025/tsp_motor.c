
#include "tsp_motor.h"

void tsp_encoder_init(void)
{
    TSP_QEI_Init();
}

void tsp_encoder_get_value(uint32_t *value){
    value[0] = TSP_QEI1_GetCount();
    value[1] = TSP_QEI2_GetCount();
}

void tsp_encoder_clear(void){
    TSP_QEI1_ResetCount();
    TSP_QEI2_ResetCount();
}

void Motor_test(void){
    while(1){
        SLEEP_HIGH();
        tsp_motor_voltage(MOTORF,2000);
        uint32_t value[2];
        tsp_encoder_get_value(value);
        tsp_encoder_clear();
        // 这里可以添加更多的测试逻辑
        // 例如打印编码器值等
        tsp_tft18_show_uint16(0, 4, value[0]);
        tsp_tft18_show_uint16(0, 5, value[1]);
    }
}