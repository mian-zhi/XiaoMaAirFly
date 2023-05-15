#include "control.h"

void fight_init()
{
	printf("System Init!\r\n");

	while(mpu_init()!=1);
	printf("mpu Init!\r\n");
	motor_init();
	oled_init();
	tim_control_init();
}




