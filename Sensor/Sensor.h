#ifndef __SENSOR_H
#define __SENSOR_H

#include "main.h"

extern struct bmi08x_dev dev;
extern struct bmi08x_sensor_data user_accel_bmi088;
extern struct bmi08x_sensor_data user_gyro_bmi088;
extern  u8 device_id;
extern  u8 MPU_Res;



void BMI088_InitFunc(void);
void MPU9250_DMP_InitFunc(void);
void VL53L0x_InitFunc(void);
void vl53l0x_test(void);
void SystemClock_Config(void);
void Dis_Filter_Window(uint16_t* Dis); 

#endif

