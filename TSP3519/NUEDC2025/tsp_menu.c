#include "tsp_menu.h"
#include "tsp_common_headfile.h"


typedef enum {
    keyUP,
    keyDOWN,
    keySTILL
} QESDir;

QESDir Scroll;

const unsigned char MenuCursor16x16[]={
    0x00,0x00,0x00,0x01,0x00,0x03,0x00,0x07,
    0x00,0x0F,0xFE,0x1F,0xFE,0x3F,0xFE,0x7F,
    0xFE,0x3F,0xFE,0x1F,0x00,0x0F,0x00,0x07,
    0x00,0x03,0x00,0x01,0x00,0x00,0x00,0x00,
};


uint8_t menu_item0[8][20]=
{
	"1.OpenLoop",
	"2.CloseLoop",
	// "1.SmartCarDemo",
	// "2.ImageProcess",
	// "3.Servo-Manual",
	// "4.Motor-Manual",
	// "5.SpdCloseLoop",
	// "6.E-Gradienter",
	// "7.RemoteContrl",
	// "8.SpeedSetting",
};





uint8_t tsp_menu_loop(void)
{
	uint8_t ItemNumber=0;
	uint8_t CmdIdx=0, CmdOk=0;
	uint8_t StatusA, StatusB;
	uint8_t tStatusA, tStatusB;

	tsp_tft18_clear(BLACK);
    
	tsp_tft18_show_str(32, 0, menu_item0[0]);
	tsp_tft18_show_str(32, 1, menu_item0[1]);
	// tsp_tft18_show_str(32, 2, menu_item0[2]);
	// tsp_tft18_show_str(32, 3, menu_item0[3]);
	// tsp_tft18_show_str(32, 4, menu_item0[4]);
	// tsp_tft18_show_str(32, 5, menu_item0[5]);
	// tsp_tft18_show_str(32, 6, menu_item0[6]);
	// tsp_tft18_show_str(32, 7, menu_item0[7]);

	show_menu_cursor(ItemNumber, WHITE);
	
	Scroll = keySTILL;

	StatusA = PHA2();   StatusB = PHB2();
	tStatusA = StatusA; tStatusB = StatusB;

	while (1)
	{
		Scroll = keySTILL;
		if (!S1())
		{
			delay_1ms(2);
			if (!S1())
				Scroll = keyUP;
			while(!S1());
		}
		if (!S2())
		{
			delay_1ms(2);
			if (!S2())
				Scroll = keyDOWN;
			while(!S2());
		}
        
		if (keyUP == Scroll)		// cursor move up
		{
			if (ItemNumber>0)
			{
				show_menu_cursor(ItemNumber--, BLACK);
				show_menu_cursor(ItemNumber, WHITE);
				tsp_cat9555_seg7_decimal(ItemNumber+1);
			}
		}
		if (keyDOWN == Scroll)	// cursor move down
		{
			if (ItemNumber<7)
			{
				show_menu_cursor(ItemNumber++, BLACK);
				show_menu_cursor(ItemNumber, WHITE);
				tsp_cat9555_seg7_decimal(ItemNumber+1);
			}
		}

		if (!S3())			// push button pressed        
		{
			delay_1ms(10);	// de-jitter
			if (!S3())
			{
				while(!S3());
				return ItemNumber;
			}
		}
	}
}

void show_menu_cursor(uint8_t ItemNumber, uint16_t color)
{
	uint8_t i,j;
	uint8_t temp1, temp2;

	for(i=0; i<16; i++)
	{
		tsp_tft18_set_region(12, ItemNumber*16+i, 28, ItemNumber*16+i);
		temp1 = MenuCursor16x16[i*2];
		temp2 = MenuCursor16x16[i*2+1];
		for(j=0; j<8; j++)
		{
			if(temp1&0x01)
				tsp_tft18_write_2byte(color);
			else
				tsp_tft18_write_2byte(BLACK);
			temp1 >>= 1;
		}
		for(j=0; j<8; j++)
		{
			if(temp2&0x01)
				tsp_tft18_write_2byte(color);
			else
				tsp_tft18_write_2byte(BLACK);
			temp2 >>= 1;
		}
	}
}