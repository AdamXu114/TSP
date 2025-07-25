#include "tsp_adc.h"

volatile bool ADC_flag = false; // ADC conversion flag
volatile uint16_t ADC_value = 0; // ADC value
/**
 * @brief Initialize ADC for CCD channels
 */
void ADC_Init(void) {//已经在SYSCFG_DL_init中调用，不需要单独调用
    // Initialize ADC sequence sampling for CCD
    DL_ADC12_startConversion(DL_ADC12_MEM_IDX_0);
}

/**
 * @brief Read and return the averaged ADC value from CCD channels.
 *        Triggers a conversion, waits for completion, retrieves the 4-channel sequence,
 *        and returns their average.
 * @return Averaged 12-bit ADC value (0 - 4095)
//  */

int CCD1_Get_AO(uint16_t *value){
    //while(!ADC_flag); // Wait for ADC conversion to complete
    ADC_flag = false; // Reset flag
    *value = DL_ADC12_getMemResult(CCD_INST,DL_ADC12_MEM_IDX_0);    //12bit, 0-4095
}
int CCD2_Get_AO(uint16_t *value){
    return ADC_ReadValue(DL_ADC12_MEM_IDX_1,value);
}
// int CCD3_Get_AO(uint16_t *value){
//     return ADC_ReadValue(DL_ADC12_MEM_IDX_2,value);
// }
// int CCD4_Get_AO(uint16_t *value){
//     return ADC_ReadValue(DL_ADC12_MEM_IDX_3,value);
// }


void ADC1_IRQHandler(void){
    // ADC1_IRQHandler();
    switch(DL_ADC12_getPendingInterrupt(CCD_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            // Sequence conversion complete
            ADC_flag = true;
            DL_ADC12_clearInterruptStatus(CCD_INST, DL_ADC12_IIDX_MEM0_RESULT_LOADED);
            break;
        case DL_ADC12_IIDX_MEM1_RESULT_LOADED:
            // Memory conversion complete
            ADC_flag = true;
            DL_ADC12_clearInterruptStatus(CCD_INST, DL_ADC12_IIDX_MEM1_RESULT_LOADED);
            break;
        default:
            // Other interrupts
            break;
    }
    // Clear interrupt flag
    
}