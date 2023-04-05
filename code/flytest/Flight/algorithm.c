#include "algorithm.h"

/**
	@brief algorithm for calulate the pose
	@param ptr of acce
				 ptr of gyro
				 ptr of pose

	@return 1 successful
*/
void algorithm_pose_acce_only(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose)
{
	vector3f_t acce;
	
	acce.x = _ptr_acce->x * ACCEL_SCALE;
	acce.y = _ptr_acce->y * ACCEL_SCALE;
	acce.z = _ptr_acce->z * ACCEL_SCALE;
	
	_ptr_pose->roll = atan2( acce.y, acce.z);
	_ptr_pose->pitch = -atan2( acce.x	, sqrt(acce.y*acce.y + acce.z*acce.z) );
	printf("{pose:%f,%f,%f}\n\r",_ptr_pose->pitch,_ptr_pose->roll,_ptr_pose->yaw);
}

/**
	@brief algorithm for calulate the pose
	@param ptr of acce
				 ptr of gyro
				 ptr of pose
				 ptr of function
	@return 1 successful
*/
void algorithm_pose(vector3i_t* _ptr_acce , 
										vector3i_t* _ptr_gyro , 
										ptr_euler_t _ptr_pose ,
										algorithm_pose_funtion_ptr _ptr_function)
{
	(*_ptr_function)(_ptr_acce,_ptr_gyro,_ptr_pose);
}

/**
	@brief algorithm for calulate the pose using EKF on quaternions
	@param ptr of acce
				 ptr of gyro
				 ptr of pose
	@return 1 successful
*/
void algorithm_pose_EKF_Quaternion(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose){
	
	//Initialize pose
	static struct quaternions4f_t q = {
		.qw = 1,
		.qx = 0,
		.qy = 0,
		.qz = 0
	};

	//Tiny time intervals. This term is determined by settings of TIM2.
	float delta_t = 0.01;

	//Read data
	vector3f_t w; vector3f_t acc;
	w.x = _ptr_gyro->x * GYRO_SCALE;
	w.y = _ptr_gyro->y * GYRO_SCALE;
	w.z = _ptr_gyro->z * GYRO_SCALE;
	acc.x = _ptr_acce->x * ACCEL_SCALE;
	acc.y = _ptr_acce->y * ACCEL_SCALE;
	acc.z = _ptr_acce->z * ACCEL_SCALE;
	
	//Time update
	float w_0 = 0;
	q.qw = q.qw + 1/2 * delta_t * (q.qw * w_0 - q.qx * w.x - q.qy * w.y - q.qz * w.z);
	q.qx = q.qx + 1/2 * delta_t * (q.qw * w.x + q.qx * w_0 - q.qy * w.z - q.qz * w.y);
	q.qy = q.qy + 1/2 * delta_t * (q.qw * w.y - q.qx * w.z - q.qy * w_0 - q.qz * w.x);
	q.qz = q.qz + 1/2 * delta_t * (q.qw * w.z + q.qx * w.y - q.qy * w.x - q.qz * w_0);

	//Normalization
	q.qw = q.qw / (q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz);
	q.qx = q.qx / (q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz);
	q.qy = q.qy / (q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz);
	q.qz = q.qz / (q.qw * q.qw + q.qx * q.qx + q.qy * q.qy + q.qz * q.qz);
	
	//quarernion -> Euler angle
	_ptr_pose->roll = atan2f( 2 * (q.qw * q.q1 + q.q2 * q.q3) , 1 - 2 * (q.q1 * q.q1 + q.q2 * q.q2) );
	_ptr_pose->pitch = asinf( 2 * (q.qw * q.q2 - q.q3 * q.q1) );
	_ptr_pose->yaw  = atan2f( 2 * (q.qw * q.q3 + q.q1 * q.q2) , 1 - 2 * (q.q2 * q.q2 + q.q3 + q.q3) );
	printf("{pose:%f,%f,%f}\n\r",_ptr_pose->pitch,_ptr_pose->roll,_ptr_pose->yaw);
}


