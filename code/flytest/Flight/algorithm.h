#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "imu.h"
#include "math.h"

/* struct of euler */
typedef struct{
	float pitch;
	float yaw;
	float roll;
}euler_t;
typedef euler_t* ptr_euler_t;

/* the type of function pointer which for calculate */
typedef void(*algorithm_pose_funtion_ptr)(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose);

/*
	the function for outsides
*/
void algorithm_pose(vector3i_t* _ptr_acce , 
										vector3i_t* _ptr_gyro , 
										ptr_euler_t _ptr_pose ,
										algorithm_pose_funtion_ptr _ptr_function);

/**
	all kinds of algorithm for calculate the pose
*/
void algorithm_pose_acce_only(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose);

void algorithm_pose_EKF_Quaternion(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose);

void algorithm_pose_ESKF_Quaternion(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose);

void quaternion_norm(quaternions4f_t* _ptr_quaternion);

#endif
