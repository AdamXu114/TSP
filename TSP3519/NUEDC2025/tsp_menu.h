#ifndef _TSP_MENU_H
#define _TSP_MENU_H

#include "tsp_common_headfile.h"

uint8_t tsp_menu_loop(void);
uint8_t tsp_menu_core(void);
uint8_t tsp_menu_board(void);
uint8_t tsp_demo_hmi(void);

void show_menu_cursor(uint8_t ItemNumber, uint16_t color);
void Ex0_gif_poweron(void);


void tsp_rtc_demo(void);
void rtc_pre_config(void);
void rtc_show_time(void);
void DrawRtcDemoScreen();
uint8_t BCD2BIN(uint8_t bcd);
uint8_t BIN2BCD(uint8_t bin);
void ShowArrow16x16(uint8_t x, uint8_t y, uint8_t no, uint16_t fColor, uint16_t bColor);
void ClearArrow16x16(uint8_t x, uint8_t y, uint16_t color);
void UpdateArrowPos(uint8_t no, uint16_t fColor, uint16_t bColor);
void UpdateSetValue(uint8_t no);

void tsp_demo_frame_hmi(void);
void tsp_demo_frame_sysinfo(void);
void tsp_sysinfo(void);

#endif