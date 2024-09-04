#ifndef __IMU_H
#define	__IMU_H

#include "main.h"
#include "bmi08x.h"
#include "bmi088.h"

#define FILTER_N            12
#define M_PI_F              3.141593f

#define THRESHOLD           0.6  // 角速率阈值 

#define IMU_KP              5.0f  // 比例增益控制加速度计和磁力计的融合
#define IMU_KI              0.005f  // 积分增益控制加速度计和磁力计的融合
#define halfT               0.0085f

extern volatile float gyro_x;
extern volatile float gyro_y;
extern volatile float gyro_z;
extern double gyro_z1;
extern float accel_x;
extern float accel_y;
extern float accel_z;

extern float Pitch;
extern float Roll;
extern float Yaw;

extern float Kp;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

extern  Quaternion quart;

void ACC_XYZ_Window_Filter(struct bmi08x_sensor_data *ACC_xyz); 
//void GYRO_XYZ_Window_Filter(struct bmi08x_sensor_data *GYRO_xyz);
float Single_Window_Filter(float Sample); //窗口滤波函数


void LPF2_ParamSet(double sample_freq, double cutoff_freq);
float LPF2_Calculate(float sample);

static float invSqrt(float x); 										   //快速计算 1/Sqrt(x)
void AHRS(float gx, float gy, float gz, float ax, float ay, float az); //四元数姿态解算
void MPU_Update(float gx, float gy, float gz, float ax, float ay, float az);


#endif
