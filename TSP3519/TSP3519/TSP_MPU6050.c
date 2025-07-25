#include "TSP_MPU6050.h"
#include "tsp_isr.h"
#include "TSP_TFT18.h"

#define Kp      10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki      0.008f                       // integral gain governs rate of convergence of gyroscope biases
#define halfT   0.001f                   //TODO： half the sample period,sapmple freq=500Hz
//TODO： 需要根据实际采样频率调整
 
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
 
static float yaw = 0;
static float pitch = 0;
static float roll = 0;
static float gyro_bias[3] = {8.0f, -18.0f, 35.0f};  // 零偏估计值
// 写数据到MPU6050寄存器
// 函数：MPU6050_WriteReg
// 功能：向MPU6050写入一个寄存器
// 参数：reg_add：寄存器地址；reg_dat：要写入的数据
// 返回值：返回写入结果
int MPU6050_WriteReg(uint8_t reg_add, uint8_t reg_dat)
{
    return mspm0_i2c_write( MPU6050_ADDRESS >> 1, reg_add, 1, &reg_dat);
}

// 函数：MPU6050_ReadReg
// 功能：从MPU6050读取一个寄存器的值
// 参数：reg_add：寄存器地址
// 返回值：返回读取到的数据
uint8_t MPU6050_ReadReg(uint8_t RegAddress) {
    uint8_t value;
    // 内部会用两次传输：写地址，然后重复 START + 读出数据
    mspm0_i2c_read(MPU6050_ADDRESS >> 1,  // 7 位地址
                       RegAddress,
                       1,
                       &value);
    return value;
}

// 从MPU6050寄存器读取数据
// 函数：MPU6050_ReadData
// 功能：从MPU6050读取多个寄存器的值
// 参数：reg_add：寄存器地址；Read：读取的数据；num：读取的寄存器个数
// 返回值：返回读取结果
int MPU6050_ReadData(uint8_t reg_add, uint8_t* Read, uint8_t num)
{
    return mspm0_i2c_read(MPU6050_ADDRESS >> 1, reg_add, num, Read);
}

// 函数：MPU6050ReadID
// 功能：读取MPU6050的ID
// 参数：无
// 如果读取失败，则返回0
uint8_t MPU6050ReadID(void)
{
   uint8_t Re = MPU6050_ReadReg(MPU6050_WHO_AM_I);		//返回WHO_AM_I寄存器的值
   if (Re != 0x68) {
         tsp_tft18_show_str(0,7,"MPU6050 Not Found");
         return 0;
   } else {
        // char str[30];
        // sprintf(str, "MPU6050 ID = %d", Re);
        // tsp_tft18_show_str(0,7, str);
        return 1;
   }
}

// 函数：MPU6050_Init
// 功能：初始化MPU6050
// 参数：无
// 返回值：无
void MPU6050_Init(void)
{
    if (!MPU6050ReadID()) {
        return ; // 如果没有找到 MPU6050，直接返回
    }
    //mspm0_i2c_enable();    // 硬件 I2C 一次性配置
    delay_1ms(100);
    /*MPU6050寄存器初始化，需要对照MPU6050手册的寄存器描述配置，此处仅配置了部分重要的寄存器*/
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);		//电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);		//电源管理寄存器2，保持默认值0，所有轴均不待机
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);		//采样率分频寄存器，配置采样率
	MPU6050_WriteReg(MPU6050_CONFIG, 0x06);			//配置寄存器，配置DLPF
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);	//陀螺仪配置寄存器，选择满量程为±2000°/s
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);	//加速度计配置寄存器，选择满量程为±16g
    delay_1ms(200);
    for(int i = 0; i < 100; i++) {
        short gyro[3];
        MPU6050ReadGyro(gyro);
        gyro_bias_update(gyro);
        delay_1ms(10);
    }
}

void MPU6050ReadAcc(short *accData)
{
    uint8_t buf[6];
    MPU6050_ReadData(MPU6050_ACCEL_XOUT_H, buf, 6);
    accData[0] = ((buf[0] << 8) | buf[1]) ; // 减去36是为了校准偏移量
    accData[1] = ((buf[2] << 8) | buf[3]) ;
    accData[2] = ((buf[4] << 8) | buf[5]) ;
    tsp_tft18_show_int16(0, 7, accData[0]); // X轴
    tsp_tft18_show_int16(40, 7, accData[1]); // Y轴
    tsp_tft18_show_int16(80, 7, accData[2]); // Z轴
}

// 函数：MPU6050ReadGyro
// 功能：读取MPU6050的陀螺仪数据
// 参数：gyroData：陀螺仪数据
// 返回值：无
void MPU6050ReadGyro(short *gyroData)
{
   uint8_t buf[6];
   if (!MPU6050_ReadData(MPU6050_GYRO_XOUT_H, buf, 6)){
       gyroData[0] = ((buf[0] << 8) | buf[1]);
       gyroData[1] = ((buf[2] << 8) | buf[3]);
       gyroData[2] = ((buf[4] << 8) | buf[5]); // 减去37是为了校准偏移量
   }
   else {
       gyroData[0] = gyro_bias[0];
       gyroData[1] = gyro_bias[1];
       gyroData[2] = gyro_bias[2];
   }
    // tsp_tft18_show_int16(0, 6, gyroData[0]); // X轴
    // tsp_tft18_show_int16(40, 6, gyroData[1]); // Y轴
    // tsp_tft18_show_int16(80, 6, gyroData[2]); // Z轴
}
void gyro_bias_update(short *gyroData){
    //当data与bias的差值小于10时，更新bias
    // if(abs(gyroData[0] - gyro_bias[0]) < 10 
    //     && abs(gyroData[1] - gyro_bias[1]) < 10 
    //     && abs(gyroData[2] - gyro_bias[2]) < 10){
        gyro_bias[0] += ((float)gyroData[0] - gyro_bias[0]) / 20;
        gyro_bias[1] += ((float)gyroData[1] - gyro_bias[1]) / 20;
        gyro_bias[2] += ((float)gyroData[2] - gyro_bias[2]) / 20;
    //  }
    // char str[30];
    // sprintf(str, "%.2f, %.2f, %.2f", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    // tsp_tft18_show_str(0, 5, str);
}
void IMU_Update(float gx, float gy, float gz, 
                float ax, float ay, float az)
{
    // 1) 归一化加速度，避免除 0
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    // 2) 估计重力方向
    float vx = 2*(q1*q3 - q0*q2);
    float vy = 2*(q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3) 计算误差（向量叉乘）
    float ex = (ay*vz - az*vy);
    float ey = (az*vx - ax*vz);
    float ez = (ax*vy - ay*vx);

    // 4) 积分误差（限幅）
    exInt += Ki * ex;
    eyInt += Ki * ey;
    ezInt += Ki * ez;
    // exInt = clamp(exInt, -imax, imax); …  

    // 5) 纠正陀螺读数
    gx += Kp * ex + exInt;
    gy += Kp * ey + eyInt;
    gz += Kp * ez + ezInt;

    // 6) 四元数微分积分
    float t0=q0, t1=q1, t2=q2, t3=q3;
    q0 += (-t1*gx - t2*gy - t3*gz) * halfT;
    q1 += ( t0*gx + t2*gz - t3*gy) * halfT;
    q2 += ( t0*gy - t1*gz + t3*gx) * halfT;
    q3 += ( t0*gz + t1*gy - t2*gx) * halfT;

    // 7) 四元数归一化
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    // 8) 计算欧拉角（°）
    roll  = atan2f(2*(q0*q1 + q2*q3),
                   1-2*(q1*q1+q2*q2))*RAD2DEG;
    pitch = asinf(2*(q0*q2 - q1*q3))*RAD2DEG;
    yaw   = atan2f(2*(q1*q2 + q0*q3),
                   1-2*(q2*q2+q3*q3))*RAD2DEG;
}
void RPY_Update(void)
{
    //使用积分来更新rpy
    short gyrodata[3] = {0};
    const float DT = 0.02f; // 20ms采样周期
    // 1) 读取原始陀螺仪数据（MPU6050ReadGyro 已在你的库里实现）
    
    MPU6050ReadGyro(gyrodata);
    //gyro_bias_update(gyrodata);
    // 2) 转换成角速度 (°/s)
    float gx = (gyrodata[0]-gyro_bias[0]) / GYROSCALE;
    float gy = (gyrodata[1]-gyro_bias[1]) / GYROSCALE;
    float gz = (gyrodata[2]-gyro_bias[2]) / GYROSCALE;
    

    // 3) 纯积分更新
    roll  += gx * DT;
    pitch += gy * DT;
    yaw   += gz * DT;
}

void MPU6050GetRPY(float *Roll, float *Pitch, float *Yaw)
{
    // short accData[3] = {0};
    // short gyroData[3] = {0};
    // MPU6050ReadAcc(accData); // 读取加速度计数据
    // MPU6050ReadGyro(gyroData); // 读取陀螺仪数据
    // 调用IMU_Update函数更新IMU数据
    //IMU_Update(gyroData[0], gyroData[1], gyroData[2], accData[0], accData[1], accData[2]); // 更新IMU
    RPY_Update(); // 更新RPY
    *Roll  = roll / 63.7f * 90.0f;
    *Pitch = pitch / 63.7f * 90.0f;
    *Yaw   = yaw / 63.7f * 90.0f;
}
