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
	@return none
*/
void algorithm_pose_EKF_Quaternion(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , ptr_euler_t _ptr_pose){
	
	// Initialize pose and omega
	static quaternions4f_t q = {
		.qw = 1,
		.qx = 0,
		.qy = 0,
		.qz = 0
	};

	static vector3f_t w_0 ={
		.x = 0,
		.y = 0,
		.z = 0
	};
	
	// time step for quaternion update 
	float delta_t = 0.02;
	
	// store data 
	vector3f_t w_T_2 , w_T;
	vector3f_t a_T_2 , a_T;
	static uint8_t flag = 0;
	flag++;

	if(flag % 2 != 0){
		w_T_2.x = _ptr_gyro->x * GYRO_SCALE;
		w_T_2.y = _ptr_gyro->y * GYRO_SCALE;
		w_T_2.z = _ptr_gyro->z * GYRO_SCALE;
		
		a_T_2.x = _ptr_acce->x * ACCEL_SCALE;
		a_T_2.y = _ptr_acce->y * ACCEL_SCALE;
		a_T_2.z = _ptr_acce->z * ACCEL_SCALE;
		
		return;
	}
	else if(flag % 2 == 0){
		w_T.x = _ptr_gyro->x * GYRO_SCALE;
		w_T.y = _ptr_gyro->y * GYRO_SCALE;
		w_T.z = _ptr_gyro->z * GYRO_SCALE;
		
		a_T.x = _ptr_acce->x * ACCEL_SCALE;
		a_T.y = _ptr_acce->y * ACCEL_SCALE;
		a_T.z = _ptr_acce->z * ACCEL_SCALE;
	}
	flag = 0;
	
	// Runge-kutta processing 
	float ww0 = 0;
	float s1[4],s2[4],s3[4],s4[4];
	quaternions4f_t q_t;
	// First Order 
	{
		s1[0] = 0.5 * (q.qw * ww0 - q.qx * w_0.x - q.qy * w_0.y - q.qz * w_0.z);
		s1[1] = 0.5 * (q.qw * w_0.x + q.qx * ww0 + q.qy * w_0.z - q.qz * w_0.y);
		s1[2] = 0.5 * (q.qw * w_0.y - q.qx * w_0.z - q.qy * ww0 + q.qz * w_0.x);
		s1[3] = 0.5 * (q.qw * w_0.z + q.qx * w_0.y - q.qy * w_0.x - q.qz * ww0);

		q_t.qw = q.qw + 0.5 * delta_t * s1[0];
		q_t.qx = q.qx + 0.5 * delta_t * s1[1];
		q_t.qy = q.qy + 0.5 * delta_t * s1[2];
		q_t.qz = q.qz + 0.5 * delta_t * s1[3];

		quaternion_norm(&q_t);
	}
	// Second Order 
	{
		s2[0] = 0.5 * (q_t.qw * ww0 - q_t.qx * w_T_2.x - q_t.qy * w_T_2.y - q_t.qz * w_T_2.z);
		s2[1] = 0.5 * (q_t.qw * w_T_2.x + q_t.qx * ww0 + q_t.qy * w_T_2.z - q_t.qz * w_T_2.y);
		s2[2] = 0.5 * (q_t.qw * w_T_2.y - q_t.qx * w_T_2.z - q_t.qy * ww0 + q_t.qz * w_T_2.x);
		s2[3] = 0.5 * (q_t.qw * w_T_2.z + q_t.qx * w_T_2.y - q_t.qy * w_T_2.x - q_t.qz * ww0);

		q_t.qw = q.qw + 0.5 * delta_t * s2[0];
		q_t.qx = q.qx + 0.5 * delta_t * s2[1];
		q_t.qy = q.qy + 0.5 * delta_t * s2[2];
		q_t.qz = q.qz + 0.5 * delta_t * s2[3];

		quaternion_norm(&q_t);
	}
	// Third Order 
	{
		s3[0] = 0.5 * (q_t.qw * ww0 - q_t.qx * w_T_2.x - q_t.qy * w_T_2.y - q_t.qz * w_T_2.z);
		s3[1] = 0.5 * (q_t.qw * w_T_2.x + q_t.qx * ww0 + q_t.qy * w_T_2.z - q_t.qz * w_T_2.y);
		s3[2] = 0.5 * (q_t.qw * w_T_2.y - q_t.qx * w_T_2.z - q_t.qy * ww0 + q_t.qz * w_T_2.x);
		s3[3] = 0.5 * (q_t.qw * w_T_2.z + q_t.qx * w_T_2.y - q_t.qy * w_T_2.x - q_t.qz * ww0);

		q_t.qw = q.qw + delta_t * s3[0];
		q_t.qx = q.qx + delta_t * s3[1];
		q_t.qy = q.qy + delta_t * s3[2];
		q_t.qz = q.qz + delta_t * s3[3];

		quaternion_norm(&q_t);
	}
	// Forth Order
	{
		s4[0] = 0.5 * (q_t.qw * ww0 - q_t.qx * w_T.x - q_t.qy * w_T.y - q_t.qz * w_T.z);
		s4[1] = 0.5 * (q_t.qw * w_T.x + q_t.qx * ww0 + q_t.qy * w_T.z - q_t.qz * w_T.y);
		s4[2] = 0.5 * (q_t.qw * w_T.y - q_t.qx * w_T.z - q_t.qy * ww0 + q_t.qz * w_T.x);
		s4[3] = 0.5 * (q_t.qw * w_T.z + q_t.qx * w_T.y - q_t.qy * w_T.x - q_t.qz * ww0);
	}

	// quaternion update  
	q.qw = q.qw + 0.1666 * delta_t * (s1[0] + 2 * s2[0] + 2 * s3[0] + s4[0]);
	q.qx = q.qx + 0.1666 * delta_t * (s1[1] + 2 * s2[1] + 2 * s3[1] + s4[1]);
	q.qy = q.qy + 0.1666 * delta_t * (s1[2] + 2 * s2[2] + 2 * s3[2] + s4[2]);
	q.qz = q.qz + 0.1666 * delta_t * (s1[3] + 2 * s2[3] + 2 * s3[3] + s4[3]);

	quaternion_norm(&q);

	// update omega 
	w_0.x = w_T.x;
	w_0.y = w_T.y;
	w_0.z = w_T.z;

	// quarernion -> Euler angle
	_ptr_pose->roll = atan2f( 2 * (q.qw * q.qx + q.qy * q.qz) , 1 - 2 * (q.qx * q.qx + q.qy * q.qy) );
	_ptr_pose->pitch = asinf( 2 * (q.qw * q.qy - q.qz * q.qx) );
	_ptr_pose->yaw  = atan2f( 2 * (q.qw * q.qz + q.qx * q.qy) , 1 - 2 * (q.qy * q.qy + q.qz * q.qz) );

	printf("{pose:%f,%f,%f}\n\r",_ptr_pose->pitch,_ptr_pose->roll,_ptr_pose->yaw);
	
}

/**
	@brief store omega for Runge-Kutta 
	@param ptr of gyro
				 ptr of gyro
				 ptr of omega_T
				 ptr of omega_T_2
				 index
	@return none
*/
void store_data(vector3i_t* _ptr_acce , vector3i_t* _ptr_gyro , vector3f_t* _ptr_omega_backup , vector3f_t* _ptr_acce_backup , uint8_t index){
	
	_ptr_acce_backup[index - 1].x = _ptr_acce->x * ACCEL_SCALE;
	_ptr_acce_backup[index - 1].y = _ptr_acce->y * ACCEL_SCALE;
	_ptr_acce_backup[index - 1].z = _ptr_acce->z * ACCEL_SCALE;
	_ptr_omega_backup[index - 1].x = _ptr_gyro->x * GYRO_SCALE;
	_ptr_omega_backup[index - 1].y = _ptr_gyro->y * GYRO_SCALE;
	_ptr_omega_backup[index - 1].z = _ptr_gyro->z * GYRO_SCALE;
	
}

/**
	@brief normalize quaternion  
	@param ptr of quaternion
	@return none
*/
void quaternion_norm(quaternions4f_t* _ptr_quaternion){

	float mod = (_ptr_quaternion->qw * _ptr_quaternion->qw  + 
					_ptr_quaternion->qx * _ptr_quaternion->qx + 
						_ptr_quaternion->qy * _ptr_quaternion->qy + 
							_ptr_quaternion->qz * _ptr_quaternion->qz);

	_ptr_quaternion->qw = _ptr_quaternion->qw / mod;
	_ptr_quaternion->qx = _ptr_quaternion->qx / mod;
	_ptr_quaternion->qy = _ptr_quaternion->qy / mod;
	_ptr_quaternion->qz = _ptr_quaternion->qz / mod;
}


