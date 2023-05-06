#ifndef __VIEWER_H
#define __VIEWER_H

#include "port.h"


/**
	private function
*/
/**
		==== hardware function ====
*/
void soft_iic_start(void);
void soft_iic_stop(void);
void soft_iic_wait_ack(void);
void soft_iic_write_byte(uint8_t _Byte);

/**
		==== oled function ====
*/
void oled_write_cmd(uint8_t _command);
void oled_write_data(uint8_t _data);
	
/*
	  ==== application ====
*/
void oled_show_single_asscii(uint8_t x,uint8_t y,uint8_t chr);
void oled_show_multi_asscii(uint8_t _col , uint8_t _page , uint8_t* _ptr_data , uint8_t _len , uint8_t _speed);



/**
	public function
*/
void oled_set_config(void);
void oled_fill_screen(unsigned char fill_Data);

void oled_show_start_info(void);
void oled_show_imu_data(void);

#endif 

