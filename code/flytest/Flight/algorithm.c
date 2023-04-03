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


