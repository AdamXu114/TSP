#include "tsp_adc.h"

/**
 * @brief Initialize ADC for CCD channels
 */
void ADC_Init(void) {//已经在SYSCFG_DL_init中调用，不需要单独调用
    // Initialize ADC sequence sampling for CCD
    SYSCFG_DL_CCD_init();
}

/**
 * @brief Read and return the averaged ADC value from CCD channels.
 *        Triggers a conversion, waits for completion, retrieves the 4-channel sequence,
 *        and returns their average.
 * @return Averaged 12-bit ADC value (0 - 4095)
 */
int ADC_ReadValue(DL_ADC12_MEM_IDX channel,uint16_t *value) {
    switch(channel){
        case DL_ADC12_MEM_IDX_2:// CCD1
            *value = DL_ADC12_getMemResult(CCD_INST, CCD_ADCMEM_CCD1_AO);
            break;
        case DL_ADC12_MEM_IDX_3:// CCD2
            *value = DL_ADC12_getMemResult(CCD_INST, CCD_ADCMEM_CCD2_AO);
            break;
        case DL_ADC12_MEM_IDX_4:// CCD3
            *value = DL_ADC12_getMemResult(CCD_INST, CCD_ADCMEM_CCD3_AO);
            break;
        case DL_ADC12_MEM_IDX_6:// CCD4
            *value = DL_ADC12_getMemResult(CCD_INST, CCD_ADCMEM_CCD4_AO);
            break;
        default:
            return 0;  // Invalid channel
    }
    return 1;  // Success
}
