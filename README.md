# 小马飞控 XiaoMaAirFly 

> 这是一个基于单片机的飞控系统，包括了电路和代码
>
> 我们的理念是：搭建一个完全开源的飞控平台，但是预留好各种底层的接口：陀螺仪数据、电机控制等，让使用者可以学习和验证各种飞控算法。



## 使用指南 Getting Started

### 使用条件 Prerequisites

在当前版本（v0.1.2）你需要：

- 安装 MDK5 
- 搭载了主控 stm32f103c8t6 芯片和 陀螺仪 mpu6050  的电路板（请看hardware中的原理图）



### 文件架构 File structure

v0.1.2 全部的飞控文件都放在 Flight 这个文件夹下：

- 系统文件 system file
  -  config.h：配置文件
  - port.c \ port.h：接口文件，所有的与底层相关的函数和接口都在这个文件中，如果需要移植飞控，则需要重写该文件
- 外设文件 Peripheral file
  - imu.c \ imu.h：陀螺仪相关的文件
- 数学文件 Math file
  - algorithm.c \ algorithm.h：各种算法的文件



### 使用示例 Usage example

目前已经完成了陀螺仪部分的代码，可以进行姿态解算方面的验证：

只需要在 algorithm.c 这个文件中，编写函数，例如：

```
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
```

然后再port.c中，修改algorithm_pose的最后一个参数为你的函数名参数即可：

```
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
			algorithm_pose(&acce , &gyro , &pose , algorithm_pose_acce_only);
		}
	}
}
```

## 如何加入 How join

我们欢迎任何人提供自己的算法到我们的平台上，只需要按照使用示例，编写一个 algorithm_pose_xx 到 algorithm文件中，即可进行验证。欢迎你push你的代码。



## 历史版本 Our Work

- V0.1.1（no commit）
  - 绘制第一版电路进行验证部分功能
  - 编写了底层代码
- V0.1.2
  - 绘制了第二版的电路，解决了SWD的问题
  - 编写了陀螺仪相关的配置代码，可以正常读取陀螺仪数据，并进行矫正和滤波
  - 开放了第一个接口，可以读取六个自由度的数据，并且开放了用于姿态解算的算法

