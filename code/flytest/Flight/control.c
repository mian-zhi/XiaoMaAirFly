#include "control.h"

void fight_init()
{
	printf("System Init!\r\n");
	oled_set_config();
	oled_fill_screen(0xff);
	HAL_Delay(500);
	oled_fill_screen(0x00);
	
	oled_show_single_asscii(0,0,'X');
	oled_show_single_asscii(6,0,'M');
	oled_show_single_asscii(12,0,'A');
	oled_show_single_asscii(18,0,'F');
	char data[] = "zengyuxiaoshagua";
	oled_show_multi_asscii(0,1,data,16);
	
	HAL_Delay(500);
	
	while(mpu_init()!=1);
	printf("mpu Init!\r\n");
}