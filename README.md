# 小马飞控 XiaoMaAirFly 

> 这是一个基于单片机的飞控系统，包括了电路和代码
>
> 我们的理念是：搭建一个完全开源的飞控平台，但是预留好各种底层的接口：陀螺仪数据、电机控制等，让使用者可以学习和验证各种飞控算法。



## 使用指南 Getting Started

### 使用条件 Prerequisites

在当前版本（V0.1.3）你需要：

- 安装 MDK5 
- 搭载了主控 stm32f103c8t6 芯片和 陀螺仪 mpu6050  的电路板（请看hardware中的原理图）



### 文件架构 File structure

V0.1.3 全部的飞控文件都放在 Flight 这个文件夹下：

- 系统文件 System file
  -  config.h：配置文件
  - port.c \ port.h：接口文件，所有的与底层相关的函数和接口都在这个文件中，如果需要移植飞控，则需要重写该文件
  - control.c \ control/h ：控制文件，负责调用各个库（update on 7th April by MianZhi）
- 外设文件 Peripheral file
  - imu.c \ imu.h：陀螺仪相关的文件
  - viewer.c \ viewer.h：可视化文件 （update on 7th April by MianZhi）
- 数学文件 Math file
  - algorithm.c \ algorithm.h：各种算法的文件



### 使用示例 Usage example

目前已经完成了陀螺仪部分的代码，可以进行姿态解算方面的验证：

只需要在 algorithm.c 这个文件中，编写函数，例如：

```c
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

```c
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
  - 编写了1.3寸的 oled 驱动代码，包括简单的字符显示函数 （update on 7th April by MianZhi）
  
- V0.1.3
  
  - 硬件更新（update on 11th May 2023 by Jekyll）
    - 更新了第三版的电路，重新设计为3.5cm * 3.5cm
    - 去除了ch340芯片、mircro-usb端口、修改串口端口为 GH1.25
    - 加入了 PPM 的 GH1.25 接口
  - 软件更新
    - 使用四阶 Runge-Kutta 法更新角速度状态，提高精度了（update on 11th May 2023 by wincent）
    - 更新了电机配置文件，并进行了测试
  
  

## 更新计划 Update plan

1. 测试新的陀螺仪模块
2. 对V0.1.3的电路板进行焊接和测试
3. 对Mahony算法进行重构和移植



## 贡献者 Contributors

- [mian-zhi](https://github.com/mian-zhi) ：发起并维护了该项目，并编写底层驱动

- [akawincent](https://github.com/akawincent) ：部署飞控算法

- [Jekyll-Dieleco](https://github.com/Jekyll-Dieleco) ：更新V0.1.3版本电路，维护硬件

