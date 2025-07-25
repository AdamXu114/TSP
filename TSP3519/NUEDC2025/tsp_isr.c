#include "tsp_isr.h"

volatile uint32_t sys_tick_counter=0;
volatile static uint32_t delay;
extern uint8_t flag_20_ms;
extern uint8_t rx_buffer[];
extern uint16_t rx_idx;
extern uint8_t rx_flag;
extern uint32_t RES_value;
void delay_1ms(uint32_t count)
{
	delay = count;
	while(0U != delay) {}
}

void SysTick_Handler()
{
	sys_tick_counter++;
	if(0U != delay) {
		delay--;
	}
	if (!(sys_tick_counter % 20))
	{
		// LED_R_TOGGLE();
		flag_20_ms = 1;
	}
}
uint32_t get_systick_counter(void)
{
	return sys_tick_counter;
}
int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = sys_tick_counter;
    return 0;
}
/*
void UART0_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART0))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART0, UART0->CPU_INT.RIS);
}

void UART1_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART1))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART1, UART1->CPU_INT.RIS);
}

void UART2_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART2))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART2, UART2->CPU_INT.RIS);
}

void UART3_IRQHandler (void)
{
	switch(DL_UART_getPendingInterrupt(UART3))
	{
		case DL_UART_IIDX_TX:
		case DL_UART_IIDX_RX:
		default:
			break;
	}
	DL_UART_clearInterruptStatus(UART3, UART3->CPU_INT.RIS);
}
*/
void UART6_IRQHandler (void)
{
	uint8_t data;

	if(!DL_UART_isRXFIFOEmpty(K230_INST))
	{
		/* read one byte from the receive data register */
		data = (uint8_t)DL_UART_Main_receiveData(K230_INST);
        if(data == '\n' || data == '\r') {
            // 处理换行符或回车符
            if(rx_idx > 0) {
                rx_buffer[rx_idx] = '\0'; // 结束字符串
                rx_idx = 0; // 重置索引
                rx_flag = 1; // 读到了一个完整的命令
                
            }
        } else if(rx_idx < 255) {
            // 确保不会溢出缓冲区
            // 将数据存入缓冲区
            rx_buffer[rx_idx++] = data;
        }
	}
}

void GROUP1_IRQHandler(void)
{
    switch(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))	{
		case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
			if (DL_GPIO_getEnabledInterruptStatus(PORTA_PORT, PORTA_PHA0_PIN)) {
				//PHA0中断
				//tsp_tft18_show_str(0, 6, "PHA0 Int");
				if(PHA0()){      // rising edge on PHA0
					if(!PHB0()) RES_value++;// low on PHB0 -> CW
					else if(RES_value>0)RES_value--;// high on PHB0 -> CCW
				}else{             // falling edge on PHA0
					if(PHB0())RES_value++;// high on PHB0 -> CW
					else if(RES_value>0)RES_value--; // low on PHB0 -> CCW
				}
				DL_GPIO_clearInterruptStatus(PORTA_PORT, PORTA_PHA0_PIN);
			}break;

		case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
			tsp_tft18_show_str(0, 6, "PHB0 Int");break;

		case DL_INTERRUPT_GROUP1_IIDX_GPIOC:
			tsp_tft18_show_str(0, 6, "PHC0 Int");break;

		default:tsp_tft18_show_str(0, 6, "Interrupt Error");break;
	}
	
}

