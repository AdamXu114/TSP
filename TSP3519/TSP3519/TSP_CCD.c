// TSP_CCD.c
#include "TSP_CCD.h"

ccd_t ccd_data_raw, ccd_data_old; // CCD 数据缓存
// 简单空循环延时，80 次大约 1μs（80MHz 时钟）
void tsp_ccd_delay(void)
{
    volatile uint16_t cnt = 80;
    while (cnt--) { asm("NOP"); }
}

void tsp_ccd_init(void)
{
    // 底层外设初始化（GPIO 时钟、多路复用、ADC）
    SYSCFG_DL_init();  
    ADC_Init();

    // 配置 SI、CLK 引脚为数字输出
    DL_GPIO_initDigitalOutput(PORTC_CCD_SI1_IOMUX);
    DL_GPIO_initDigitalOutput(PORTC_CCD_SI2_IOMUX);  // 若使用双 SI
    DL_GPIO_initDigitalOutput(PORTB_CCD_CLK1_IOMUX);
    DL_GPIO_initDigitalOutput(PORTC_CCD_CLK2_IOMUX); // 若使用双 TAP
}

/** 仅拉一次 SI 脉冲，复位电荷 */
static void tsp_ccd_trigger_SI(void)
{
    // SI = 1
    DL_GPIO_setPins(PORTC_PORT, PORTC_CCD_SI1_PIN | PORTC_CCD_SI2_PIN);
    tsp_ccd_delay();
    // SI = 0
    DL_GPIO_clearPins(PORTC_PORT, PORTC_CCD_SI1_PIN | PORTC_CCD_SI2_PIN);
}

/** 拉一次 CLK 脉冲，用于读出/迭代像素 */
static void tsp_ccd_pulse_CLK(void)
{
    DL_GPIO_setPins(PORTB_PORT, PORTB_CCD_CLK1_PIN);
    DL_GPIO_setPins(PORTC_PORT, PORTC_CCD_CLK2_PIN);
    tsp_ccd_delay();
    DL_GPIO_clearPins(PORTB_PORT, PORTB_CCD_CLK1_PIN);
    DL_GPIO_clearPins(PORTC_PORT, PORTC_CCD_CLK2_PIN);
    tsp_ccd_delay();
}

void tsp_ccd_flush(void)
{
    // 1. 复位脉冲
    tsp_ccd_trigger_SI();
    // 2. 首个时钟：启动转移
    tsp_ccd_pulse_CLK();
    // 3. 再来 128 个时钟，丢弃管脚上的电荷输出
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        tsp_ccd_pulse_CLK();
    }
}

bool tsp_ccd_snapshot(ccd_t buf)
{
    uint16_t val;

    // 1) 清空上一帧
    tsp_ccd_flush();

    // 2) 等待新帧积分（根据环境光强度调整 ms）
    delay_1ms(10);

    // 3) 触发 SI, CLK 初始脉冲，开始读第 0 像素
    tsp_ccd_trigger_SI();
    DL_GPIO_clearPins(PORTB_PORT, PORTB_CCD_CLK1_PIN);
    tsp_ccd_delay();

    // 4) 依次读出 CCD_PIXEL_COUNT 个像素
    for (uint16_t i = 0; i < CCD_PIXEL_COUNT; i++) {
        // 上升沿出电荷
        DL_GPIO_setPins(PORTB_PORT, PORTB_CCD_CLK1_PIN);
        tsp_ccd_delay();

        // 读 ADC
        if (!ADC_ReadValue(DL_ADC12_MEM_IDX_2, &val)) {
            return false;
        }
        buf[i] = val;

        // 下降沿，为下一像素做准备
        DL_GPIO_clearPins(PORTB_PORT, PORTB_CCD_CLK1_PIN);
        tsp_ccd_delay();
    }

    // 5) 额外再打一拍 CLK 以终止输出
    tsp_ccd_pulse_CLK();

    return true;
}


void tsp_ccd_show(ccd_t data)
{
    uint8_t i=0;

    for(i=0; i<CCD_PIXEL_COUNT; i++)
	{
        tsp_tft18_draw_pixel(32+i, 128-(ccd_data_old[i]>>6), GRAY1);
        tsp_tft18_draw_pixel(32+i, 128-(data[i]>>6), BLUE);
        ccd_data_old[i] = data[i];
	}
}

void tsp_demo_frame_ccd(void)
{
    tsp_tft18_show_str_color(1, 0, "ExpT:       Max:    ", WHITE, BLACK);
    tsp_tft18_show_str_color(1, 1, "Mode:       Min:    ", WHITE, BLACK);
    tsp_tft18_show_str_color(1, 2, "            Avg:    ", WHITE, BLACK);

    // window for TSL1401 waveform
    tsp_tft18_draw_frame(31, 64, 128, 64, BLUE);
    tsp_tft18_draw_block(32, 65, 128, 63, GRAY1);
}


void CCD_test(void)
{
  	uint8_t index;
	
  	// initialize LCD
	// hsp_spi_init();
	// hsp_tft18_init();
	tsp_tft18_clear(BLACK);
	
	// initialize ADC/CCD
	// tsp_ccd_init();
	tsp_demo_frame_ccd();

	for(index=0; index<128; index++)
		ccd_data_raw[index] = (index<<5);
	tsp_ccd_show(ccd_data_raw);
	while(1)
	{
		if(tsp_ccd_snapshot(ccd_data_raw)){
			tsp_ccd_show(ccd_data_raw);
        }
        delay_1ms(100);
    }
	
}