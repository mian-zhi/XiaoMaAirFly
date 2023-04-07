#include "port.h"
#include "imu.h"
#include "algorithm.h"
/**
	
*/


/**
	@brief read single data from mpu6050 register and write to the data
	@param uint8_t   slave_address
				 uint8_t   reg_address
				 uint8_t*  ptr_reg_data
	@return
			1 sccessd
			0 fail
*/
int i2c_mpu_single_read(uint8_t slave_address, uint8_t reg_address, uint8_t *ptr_data)
{
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, ptr_data, 1 , 9999) != HAL_OK)
		return -1;
	return 0;
}

/**
	@brief write single data to the mpu6050 register
	@param uint8_t   slave_address
				 uint8_t   reg_address
				 uint8_t   reg_data
	@return
			1 sccessd
			0 fail
*/
int i2c_mpu_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
{
	if (HAL_I2C_Mem_Write(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, &reg_data, 1 , 9999) != HAL_OK)
		return -1;
	return 0;
}

/**
	@brief read the multi data from the Continuous registers of mpu6050
	@param uint8_t   slave_address
				 uint8_t   reg_address
				 uint8_t*  ptr_data
				 uint8_t   lenth
	@return
			1 sccessd
			0 fail
*/
int i2c_mpu_multi_read(uint8_t slave_address, uint8_t reg_address, uint8_t *ptr_data, uint8_t lenth)
{
	if (HAL_I2C_Mem_Read(&hi2c1, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, ptr_data, lenth,9999) != HAL_OK)
		return -1;
	return 0;
}


/**
	@brief write single data to the oled register
	@param uint8_t   slave_address
				 uint8_t   reg_address
				 uint8_t   reg_data
	@return
			1 sccessd
			0 fail
*/
//int i2c_oled_single_write(uint8_t slave_address, uint8_t reg_address, uint8_t reg_data)
//{
//	if (HAL_I2C_Mem_Write(&hi2c2, slave_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, &reg_data, 1 , 9999) != HAL_OK)
//		return -1;
//	return 0;
//}


/**
	@brief tim2
	@param 
	@return
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim2.Instance) //5ms
	{	
		static int16_t tim_delay = 0;
		static vector3i_t acce , gyro ;
		static euler_t pose;
		tim_delay++;
		if(tim_delay%2 == 0) // 1ms * 500 = 500ms
		{
			mpu_data_update(&acce , &gyro ,1 ,1 ,0);
			algorithm_pose(&acce , &gyro , &pose , algorithm_pose_EKF_Quaternion);
		}
	}
}


void delay_ms(uint32_t _time){
	HAL_Delay(_time);
}


/**
	@brief Toggle the led
	@param void
	@return void
*/
void gpio_led_red_toggle()
{
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
}

/**
	@brief turn on the red led
	@param void
	@return void
*/
void gpio_led_red_turn_on()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

/**
	@brief turn on the red led
	@param void
	@return void
*/
void gpio_led_red_turn_off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}



/* SCL -- PB10 */
void gpio_iic_scl_set(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}
void gpio_iic_scl_reset(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}



/* SDA -- PB11 */
void gpio_iic_sda_set(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}
void gpio_iic_sda_reset(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
}





