#ifndef __KALMAN_H
#define __KALMAN_H 

#include "tsp_common_headfile.h"
//¿¨¶ûÂüÂË²¨Ëã·¨¿â



void Angle_Calcu(void);
void Kalman_Filter_X(float Accel,float Gyro);
void Kalman_Filter_Y(float Accel,float Gyro);


#endif
