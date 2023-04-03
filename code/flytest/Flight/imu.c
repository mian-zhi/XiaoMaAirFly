#include "imu.h"

static IMU_STATIC_PARAMETERS imu_static_param;

/*
	@brief test the mpu6050
	@param void
	
	@return
			1 sccessd
			0 fail
*/
char mpu_test(void)
{
	uint8_t who_am_i;
	i2c_mpu_single_read(MPU6050_ADR, WHO_AM_I, &who_am_i);
	if(who_am_i == MPU6050_ADR) return 1;               //if the mpu 6050 exist return 1
  else return 0;
}

/*
	@brief init the mpu6050
	@param void
	
	@return void
*/
void mpu_config(void)
{
	i2c_mpu_single_write(MPU6050_ADR, PWR_MGMT_1, 0x00);   //turn off interrupt and resleep
	i2c_mpu_single_write(MPU6050_ADR, SMPLRT_DIV, 0x00);   //set sampling rate
	i2c_mpu_single_write(MPU6050_ADR, MPU_CONFIG, 0x00);   //low-pass filtering
	i2c_mpu_single_write(MPU6050_ADR, GYRO_CONFIG, 0x08);  //set gyro full scale 500deg/s
	i2c_mpu_single_write(MPU6050_ADR, ACCEL_CONFIG, 0x10); //set scale full scale 4096 LSB/g
}

/**
	@brief read the int16 data of acc from the mpu6050
	@param ptr of Vector3i_t
	
	@return void
*/
void mpu_read_acce_raw(vector3i_t* _ptr_acce)
{
    int ret;
    uint8_t retry;
    uint8_t buffer[6];

    retry = 5;
    while (retry--) {
        ret = i2c_mpu_multi_read(MPU6050_ADR, ACCEL_XOUT_H, buffer, 6);
        if (ret == 0)
            break;
    }
	
    _ptr_acce->x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    _ptr_acce->y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
    _ptr_acce->z = ((((int16_t)buffer[4]) << 8) | buffer[5]);
}

/**
	@brief read the int16 data of gyro from the mpu6050
	@param ptr of Vector3i_t
	
	@return void
*/
void mpu_read_gyro_raw(vector3i_t* _ptr_gyro)
{
    int ret;
    uint8_t retry;
    uint8_t buffer[6];

    retry = 5;
    while (retry--) {
        ret = i2c_mpu_multi_read(MPU6050_ADR, GYRO_XOUT_H, buffer, 6);
        if (ret == 0)
            break;
    }

    _ptr_gyro->x = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    _ptr_gyro->y = ((((int16_t)buffer[2]) << 8) | buffer[3]);
    _ptr_gyro->z = ((((int16_t)buffer[4]) << 8) | buffer[5]);
}



/**
	@brief read the data of temp from the mpu6050
	@param ptr of float
	
	@return void
*/
void mpu_read_temp(float* ptr_temp)
{
    int ret;
    uint8_t retry;
    uint8_t buffer[2];
    int16_t temperature_temp;

    retry = 5;
    while (retry--) {
        ret = i2c_mpu_multi_read(MPU6050_ADR, TEMP_OUT_H, buffer, 2);
        if (ret == 0)
            break;
    }
    
    temperature_temp = ((((int16_t)buffer[0]) << 8) | buffer[1]);
    *ptr_temp = 36.53f + (float)temperature_temp / 340.f;
}


/**
	Calibration
*/

/**
	@brief get the offset of acc
	@param void

	@return void
*/
void mpu_acce_get_offset()
{
	int num_samples;
	vector3l_t acce_sample_sum;
	vector3i_t acce_raw_data;
	
	acce_sample_sum.x = 0;
	acce_sample_sum.y = 0;
	acce_sample_sum.z = 0;
	
	printf("Acc Calibration Start\n\r");
	for(num_samples = 0 ; num_samples < 100 ; num_samples++){
		mpu_read_acce_raw(&acce_raw_data);//读取原始数据
		acce_sample_sum.x += acce_raw_data.x;
		acce_sample_sum.y += acce_raw_data.y;
		acce_sample_sum.z += (acce_raw_data.z-4096);
		printf("acce_sample_sum:%d,%d,%d\n\r",acce_sample_sum.x,acce_sample_sum.y,acce_sample_sum.z);
		delay_ms(5);
	}
	
	imu_static_param.acce_offset.x = acce_sample_sum.x / num_samples;
	imu_static_param.acce_offset.y = acce_sample_sum.y / num_samples;
	imu_static_param.acce_offset.z = acce_sample_sum.z / num_samples;
	
	printf("Acc_RawData_Offset:%d,%d,%d\n\r", imu_static_param.acce_offset.x,
																						imu_static_param.acce_offset.y,
																						imu_static_param.acce_offset.z);
	printf("Acc Calibration Stop\n\r");
}


/**
	@brief get the offset of gyro
	@param void

	@return void
*/
void mpu_gyro_get_offset()
{
	int num_samples;
	vector3l_t gyro_sample_sum;
	vector3i_t gyro_raw_data;
	
	gyro_sample_sum.x = 0;
	gyro_sample_sum.y = 0;
	gyro_sample_sum.z = 0;
	
	printf("Gyro Calibration Start\n\r");
	for(num_samples = 0 ; num_samples < 100 ; num_samples++){
		mpu_read_gyro_raw(&gyro_raw_data);
		gyro_sample_sum.x += gyro_raw_data.x;
		gyro_sample_sum.y += gyro_raw_data.y;
		gyro_sample_sum.z += gyro_raw_data.z;
		printf("acce_sample_sum:%d,%d,%d\n\r",gyro_sample_sum.x,gyro_sample_sum.y,gyro_sample_sum.z);
		delay_ms(5);
	}
	
	imu_static_param.gyro_offset.x = gyro_sample_sum.x / num_samples;
	imu_static_param.gyro_offset.y = gyro_sample_sum.y / num_samples;
	imu_static_param.gyro_offset.z = gyro_sample_sum.z / num_samples;
	
	printf("Gyro_RawData_Offset:%d,%d,%d\n\r", imu_static_param.gyro_offset.x,
																						imu_static_param.gyro_offset.y,
																						imu_static_param.gyro_offset.z);
	printf("Gyro Calibration Stop\n\r");
}

/**
	@brief calibrate the acce
	@param void

	@return void
*/
void mpu_acce_calibrate(vector3i_t* _ptr_acce)
{
		_ptr_acce->x = _ptr_acce->x - imu_static_param.acce_offset.x;
		_ptr_acce->y = _ptr_acce->y - imu_static_param.acce_offset.y;
		_ptr_acce->z = _ptr_acce->z - imu_static_param.acce_offset.z;
}

/**
	@brief calibrate the gyro
	@param void

	@return void
*/
void mpu_gyro_calibrate(vector3i_t* _ptr_gyro)
{
		_ptr_gyro->x = _ptr_gyro->x - imu_static_param.gyro_offset.x;
		_ptr_gyro->y = _ptr_gyro->y - imu_static_param.gyro_offset.y;
		_ptr_gyro->z = _ptr_gyro->z - imu_static_param.gyro_offset.z;
}



/**
		Filetr
*/

/**
	@brief butterworth filter
	@param current input
				 ptr of the butterworth data
				 ptr of the butterworth parameters
	
	@return butterworth filter output
*/
float butterworth_filter( float _input, 
													ptr_butter_data _ptr_data, 
													ptr_butter_parameters _ptr_parameters){
	
	/* current input */
	_ptr_data->input[2] = _input;

	/* calulate the output of the filter */
	_ptr_data->output[2] =
	_ptr_parameters->b[0] * _ptr_data->input[2]
		+ _ptr_parameters->b[1] * _ptr_data->input[1]
		+ _ptr_parameters->b[2] * _ptr_data->input[0]
		- _ptr_parameters->a[1] * _ptr_data->output[1]
		- _ptr_parameters->a[2] * _ptr_data->output[0];

	/* save input */
	_ptr_data->input[0] = _ptr_data->input[1];
	_ptr_data->input[1] = _ptr_data->input[2];

	/* save output */
	_ptr_data->output[0] = _ptr_data->output[1];
	_ptr_data->output[1] = _ptr_data->output[2];

	/* return output */														
	return (_ptr_data->output[2]);
}


/**
	@brief base on the cut-off frequent to calculate the parameters of the butterworth filter
	@param sample frequent
				 cut-off frequent
				 ptr of the parameters
	
	@return butterworth filter output
*/
void calculate_butterworth_parameters(float _sample_frequent, 
													float _cutoff_frequent, 
													ptr_butter_parameters _ptr_parameters){
	float fr = _sample_frequent / _cutoff_frequent;
	float ohm = tanf(3.141592 / fr);
	float c = 1.0f + 2.0f * cosf(3.141592 / 4.0f) * ohm + ohm * ohm;
	if (_cutoff_frequent <= 0.0f)
		return;
	_ptr_parameters->b[0] = ohm * ohm / c;
	_ptr_parameters->b[1] = 2.0f * _ptr_parameters->b[0];
	_ptr_parameters->b[2] = _ptr_parameters->b[0];
	_ptr_parameters->a[0] = 1.0f;
	_ptr_parameters->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
	_ptr_parameters->a[2] = (1.0f - 2.0f * cosf(3.141592 / 4.0f) * ohm + ohm * ohm) / c;
}
/**
	@brief fiter
	@param ptr of the date
	
	@return butterworth filter output
*/
void mpu_acc_filter(vector3i_t* _ptr_acce)
{
	_ptr_acce->x = butterworth_filter(_ptr_acce->x ,
															&(imu_static_param.acce_low_pass_fiter_data[0]),
															&(imu_static_param.acce_low_pass_fiter_param));
	_ptr_acce->y = butterworth_filter(_ptr_acce->y ,
															&(imu_static_param.acce_low_pass_fiter_data[1]),
															&(imu_static_param.acce_low_pass_fiter_param));
	_ptr_acce->z = butterworth_filter(_ptr_acce->z ,
															&(imu_static_param.acce_low_pass_fiter_data[2]),
															&(imu_static_param.acce_low_pass_fiter_param));
}


/**
	port to outsides
*/

/**
	@brief init the mpu
	@param void

	@return 1 successful
*/
char mpu_init(void)
{
	while(mpu_test()!=1); 
	mpu_config();
	mpu_acce_get_offset();
	mpu_gyro_get_offset();
	calculate_butterworth_parameters(SAMPLE_FREQUENCE,
																	 10 , 
																	 &(imu_static_param.acce_low_pass_fiter_param));
	return 1;
}

/**
	@brief update date of the mpu
	@param ptr of acce
				 ptr of gyro
				 whether to enable calibrate
				 whether to enable filter
				 whether to enable print

	@return 1 successful
*/
char mpu_data_update(vector3i_t* _ptr_acce , 
										 vector3i_t* _ptr_gyro , 
										 char _bool_calibrate,
										 char _bool_filter,
										 char _bool_print_data){
	/* read the raw data from mpu6050 */
	mpu_read_acce_raw(_ptr_acce);
	mpu_read_acce_raw(_ptr_acce);
	
	/* calibrate the raw data */
	if(_bool_calibrate == 1)
	{
		mpu_acce_calibrate(_ptr_acce);
		mpu_gyro_calibrate(_ptr_acce);
	}
	if(_bool_print_data == 1)
		printf("{ACC_raw:%d,%d,%d}\n\r",_ptr_acce->x,_ptr_acce->y,_ptr_acce->z);
	
	/* filter */
	if(_bool_filter == 1)
	{
		mpu_acc_filter(_ptr_acce);
	}
	if(_bool_print_data == 1)
		printf("{ACC_filter:%d,%d,%d}\n\r",_ptr_acce->x,_ptr_acce->y,_ptr_acce->z);
	return 1;
}


