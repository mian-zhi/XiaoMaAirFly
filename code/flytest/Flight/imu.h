#ifndef __IMU_H
#define __IMU_H

#include "port.h"
#include "math.h"


#define SAMPLE_FREQUENCE 100

//I2C address of device
#define MPU6050_ADR 0x68

//MPU6050 Address of regs
#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	WHO_AM_I		0x75
#define USER_CTRL		0x6A
#define INT_PIN_CFG		0x37


//Gravitational acceleration
#define GRAVITY_MSS 9.80665f

//scale of gyroscope
#define GYRO_SCALE_2000  (0.0174532f / 16.4f)
#define GYRO_SCALE_1000  (0.0174532f / 32.8f)
#define GYRO_SCALE_500   (0.0174532f / 65.5f)
#define GYRO_SCALE_250   (0.0174532f / 131f)

//scale of accelerometer
#define ACCEL_SCALE_16G   (GRAVITY_MSS / 2048.0f)
#define ACCEL_SCALE_8G    (GRAVITY_MSS / 4096.0f)
#define ACCEL_SCALE_4G    (GRAVITY_MSS / 8192.0f)
#define ACCEL_SCALE_2G    (GRAVITY_MSS / 16384.0f)

#define ACCEL_MAX_1G       4096.0f
#define ACCEL_SCALE        ACCEL_SCALE_8G
#define GYRO_SCALE         GYRO_SCALE_500

#define GYRO_CALIBRATION_COFF 0.0152672f
	
char mpu_test(void);
void mpu_config(void);
void mpu_read_acce_raw(vector3i_t* _ptr_acce);
void mpu_read_gyro_raw(vector3i_t* _ptr_gyro);
void mpu_read_temp(float* ptr_temp);

/**
	Calibaration
*/
void mpu_acce_get_offset(void);
void mpu_gyro_get_offset(void);
void mpu_calibrate(void);


/**
	Filter
*/

typedef struct{
 float input[3];
 float output[3];
}butter_data_t;
typedef butter_data_t* ptr_butter_data;

typedef struct{
  float a[3];
  float b[3];
}butter_parameters_t;
typedef butter_parameters_t* ptr_butter_parameters;

//function
float butterworth_filter(float _input, ptr_butter_data _ptr_data, ptr_butter_parameters _ptr_parameters);
void calculate_butterworth_parameters(float _sample_frequent, float _cutoff_frequent, ptr_butter_parameters _ptr_parameters);
void mpu_acc_filter(vector3i_t* _ptr_acce);



/**
	MPU6050 GLOBAL
*/
typedef struct{
	/* calibration */
	vector3l_t acce_offset;
	vector3l_t gyro_offset;
	
	/* fiter */
	butter_data_t acce_low_pass_fiter_data[3];
	butter_parameters_t acce_low_pass_fiter_param;
	
	
}IMU_STATIC_PARAMETERS;

char mpu_init(void);
char mpu_data_update(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , char _bool_calibrate, char _bool_filter , char _bool_print_data);

void mpu_get_acce_offest(vector3l_t* _ptr_offset);
void mpu_get_gyro_offest(vector3l_t* _ptr_offset);

#endif
