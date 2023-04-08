#include "control.h"

void fight_init()
{
	printf("System Init!\r\n");
	oled_set_config();
	oled_fill_screen(0xff);
	HAL_Delay(500);
	oled_fill_screen(0x00);
	
	oled_show_start_info();
	
	while(mpu_init()!=1);
	printf("mpu Init!\r\n");

	oled_show_imu_data();
	
	vector3l_t acce_offset , gyro_offset;
	mpu_get_acce_offest(&acce_offset);
	mpu_get_gyro_offest(&gyro_offset);

	/**
		acc offset display
	*/
	uint8_t accex[5] ;
	int_to_5_char(acce_offset.x , accex , 0);
	oled_show_multi_asscii(3*6,1,accex,5,0);
	
	uint8_t accey[5] ;
	int_to_5_char(acce_offset.y , accey , 0);
	oled_show_multi_asscii((3+6)*6,1,accey,5,0);
	
	uint8_t accez[5] ;
	int_to_5_char(acce_offset.z , accez , 0);
	oled_show_multi_asscii((3+6+6)*6,1,accez,5,0);
	
	/**
		gyro offset display
	*/
	uint8_t gyrox[5] ;
	int_to_5_char(gyro_offset.x , gyrox , 0);
	oled_show_multi_asscii(3*6,2,gyrox,5,0);
	
	uint8_t gyroy[5] ;
	int_to_5_char(gyro_offset.y , gyroy , 0);
	oled_show_multi_asscii((3+6)*6,2,gyroy,5,0);
	
	uint8_t gyroz[5] ;
	int_to_5_char(gyro_offset.z , gyroz , 0);
	oled_show_multi_asscii((3+6+6)*6,2,gyroz,5,0);

	

}

void int_to_5_char(int32_t _int , uint8_t* _ptr_char , uint8_t _offset_ptr)
{
	uint8_t byte , iter;
	if(_int < 0)
	{
		_int = -_int;
		_ptr_char[_offset_ptr] = '-';
	}
	else _ptr_char[_offset_ptr] = '+';
	for(iter = 0 ; iter < 4 ; iter++)
	{
		byte = _int % 10;
		_int = _int / 10;
		_ptr_char[4-iter+_offset_ptr] = byte + '0';
	}
}



