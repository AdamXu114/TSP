#ifndef TSP_ADC_H
#define TSP_ADC_H

#include "ti_msp_dl_config.h"
#include "tsp_gpio.h"

void ADC_Init(void);
int ADC_ReadValue(DL_ADC12_MEM_IDX channel,uint16_t *value);


#endif /* TSP_ADC_H */