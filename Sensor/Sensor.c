#include "sensor.h"
#include "i2c.h"


/***************************************各类变量定义****************************************/
struct bmi08x_sensor_data user_accel_bmi088;
struct bmi08x_sensor_data user_gyro_bmi088;
struct bmi08x_dev dev = {
        .accel_id = BMI08X_ACCEL_I2C_ADDR_SECONDARY,
        .gyro_id = BMI08X_GYRO_I2C_ADDR_SECONDARY,
        .intf = BMI08X_I2C_INTF,  
        .read = &stm32_i2c_read, 
        .write = &stm32_i2c_write, 
        .delay_ms = &HAL_Delay
};

u8 device_id;
u8 MPU_Res;

void BMI088_InitFunc(void)
{
	int8_t IMU_Res;//记录IMU初始化状态
	uint8_t data = 0;
	IMU_Res = bmi088_init(&dev);	
	if(IMU_Res == BMI08X_OK) 
	{
		IMU_Res = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev);
		if(IMU_Res == BMI08X_OK) 
		{
			IMU_Res = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
		}
	}
	else
	{
		printf("BMI088 Initial Not OK!!!\r\n");
		while(1);
	}
	IMU_Res = bmi08a_soft_reset(&dev);
	if (IMU_Res != BMI08X_OK)
	{
		printf("BMI088 RESET Not OK!!!\r\n");
		while(1);
	}
	IMU_Res = bmi08a_get_power_mode(&dev);
	IMU_Res = bmi08a_get_meas_conf(&dev);
	dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
	dev.accel_cfg.odr = BMI08X_ACCEL_ODR_200_HZ;
	dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
	dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;
	IMU_Res = bmi08a_set_power_mode(&dev);
	IMU_Res = bmi08a_set_meas_conf(&dev);
	dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;	
	IMU_Res = bmi08g_set_power_mode(&dev);
	dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;
	dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
	dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;	
	IMU_Res = bmi08g_set_meas_conf(&dev);
	printf("BMI088 Initial OK!!!\r\n");
}

void MPU9250_DMP_InitFunc(void)
{
	MPU_Res = MPU_Read_Len(MPU9250_ADDR, MPU_DEVICE_ID_REG, 1, &device_id);	// 判断IIC实物接线是否有问题 		
	if(MPU_Res)																// 同时需要判断MPU9250		
	{																		// AD0引脚是否接错
		printf("Read Error:%d\r\n",MPU_Res);
		while(1);
	}
	MPU_Res = mpu_dmp_init();
	while(1)
	{
		if(MPU_Res)		
		{
			printf("DMP Error:%d\r\n",MPU_Res);			
		}
		else
			break;
		HAL_Delay(100);
	}
	printf("MPU_DMP Init Succeed\r\n");
}


void VL53L0x_InitFunc(void)
{	
	//使用片选信号启动第一个tof
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET  );	
	HAL_Delay(100);	//等待，确保tof启动
	while(vl53l0x_init(&vl53l0x_dev[Axis_X],Xshut_Pin_X))//使用默认地址初始化第一个tof
	{
		printf("Xaxis_VL53L0x Error!!!\n\r");
		HAL_Delay(100);
	}
	printf("Xaxis_VL53L0X Init OK\r\n");	
	vl53l0x_Addr_set(&vl53l0x_dev[Axis_X],TOF_X_ADDR);
	printf("Addr:%#x\r\n",vl53l0x_dev[Axis_X].I2cDevAddr);	
	
	//启动第二个TOF
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(100);	//等待，确保tof启动
	
	//使用默认地址初始化第二个tof
	while(vl53l0x_init(&vl53l0x_dev[Axis_Y],Xshut_Pin_Y))//vl53l0x初始化
	{
		printf("Yaxis_VL53L0x Error!!!\n\r");
		HAL_Delay(100);
	}
	printf("Yaxis_VL53L0X Init OK\r\n");	
	vl53l0x_Addr_set(&vl53l0x_dev[Axis_Y],TOF_Y_ADDR);
	printf("Addr:%#x\r\n",vl53l0x_dev[Axis_Y].I2cDevAddr);
	

//	//启动第三个TOF
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
	HAL_Delay(100);	//等待，确保tof启动	
	
	//使用默认地址初始化第三个tof
	while(vl53l0x_init(&vl53l0x_dev[Axis_Z],Xshut_Pin_Z))//vl53l0x初始化
	{
		printf("Zaxis_VL53L0x Error!!!\n\r");
		HAL_Delay(100);
	}
	printf("Zaxis_VL53L0X Init OK\r\n");
	
	//修改第三个tof的iic操作地址
	vl53l0x_Addr_set(&vl53l0x_dev[Axis_Z],TOF_Z_ADDR);
	printf("Addr:%#x\r\n",vl53l0x_dev[Axis_Z].I2cDevAddr);
	
	//修改TOF读取模式，现在是默认模式，可以在宏定义中找到别的模式
	if(VL53L0X_ERROR_NONE == vl53l0x_set_mode(&vl53l0x_dev[Axis_X],HIGH_SPEED)) 
		printf("Xaxis_VL53L0X MODE SET OK\r\n");
	
	if(VL53L0X_ERROR_NONE == vl53l0x_set_mode(&vl53l0x_dev[Axis_Y],HIGH_SPEED)) 
		printf("Yaxis_VL53L0X MODE SET OK\r\n");
	
	if(VL53L0X_ERROR_NONE == vl53l0x_set_mode(&vl53l0x_dev[Axis_Z],HIGH_SPEED)) 
		printf("Zaxis_VL53L0X MODE SET OK\r\n");

}

//VL53L0X测试程序
void vl53l0x_test(void)
{   
	VL53L0X_Error status = 0; 
	static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
		  	 
	status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_X],Axis_X,&vl53l0x_data[Axis_X],buf);
	status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_Y],Axis_Y,&vl53l0x_data[Axis_Y],buf);
	status = vl53l0x_start_single_test(&vl53l0x_dev[Axis_Z],Axis_Z,&vl53l0x_data[Axis_Z],buf);
	Dis_Filter_Window(Distance_data);
	 
	if(status == VL53L0X_ERROR_NONE)
	{
		printf("%d,%d,%d\r\n",Distance_data[Axis_X],Distance_data[Axis_Y],Distance_data[Axis_Z]);
	}
	else
		printf("Status:%d\r\n",status);	
	
}


void Dis_Filter_Window(uint16_t* Dis) 
{
    static uint16_t filter_buf[3][FILTER_N + 1] = {0};
    int i = 0,j = 0;    
    uint16_t filter_sum[3] = {0};
    filter_buf[0][FILTER_N] = Dis[0];
	filter_buf[1][FILTER_N] = Dis[1];
	filter_buf[2][FILTER_N] = Dis[2];
	
    for(j = 0; j < 3;j++)
	{
		for(i = 0; i < FILTER_N; i++) 
		{
			filter_buf[j][i] = filter_buf[j][i + 1]; 
			filter_sum[j] += filter_buf[j][i];
		}
		Dis[j] = (uint16_t)(filter_sum[j] / FILTER_N);
	}
}
