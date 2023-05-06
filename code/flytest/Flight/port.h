#ifndef __PORT_H
#define __PORT_H

#include "config.h"
#include "string.h"
/*
	inclde the usart.h
*/
#include "usart.h"


/*
	inclde the iic.h
*/
#include "i2c.h"
int i2c_mpu_single_read(  uint8_t slave_address,  uint8_t reg_address,  uint8_t *ptr_data);
int i2c_mpu_single_write( uint8_t slave_address,  uint8_t reg_address,  uint8_t  reg_data);
int i2c_mpu_multi_read(   uint8_t slave_address,  uint8_t reg_address,  uint8_t *ptr_data, uint8_t lenth);



/*
	inclde the tim.h
*/
#include "tim.h"
void delay_ms(uint32_t _time);


/*
	inclde the gpio.h
*/
#include "gpio.h"
void gpio_led_red_turn_on(void);
void gpio_led_red_turn_off(void);
void gpio_led_red_toggle(void);


/* soft i2c gpio SCL - PB10  SDA - PB11 */
void gpio_iic_scl_set(void);
void gpio_iic_scl_reset(void);


void gpio_iic_sda_set(void);
void gpio_iic_sda_reset(void);

#endif
