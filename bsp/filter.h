#ifndef __FILTER_H__
#define __FILTER_H__

#include "math.h"
#include "main.h"

typedef struct
{
	float Q_angle; // 过程噪声-角度分量：越大越相信姿态变化会更快。
	float Q_bias;  // 过程噪声-温漂分量：越大越快跟踪零偏漂移。
	float R;       // 测量噪声：越大越不信任观测值。
	float P[2][2]; // 误差协方差矩阵：保存当前估计不确定度。
	float dt;      // 积分时间：滤波周期，单位通常是 s。
	float K1, K2;  // 卡尔曼增益：分别对应角度和零偏的修正力度。

	float Angle;     // 角度估计值，单位通常为度。
	float Gyro_bias; // 陀螺仪静差估计值，单位与角速度输入一致。

}KF_t;

float Mahony_Filter(float gyro, float acc);
float Kalman_Filter(KF_t *kf, float obsValue, float ut);

extern KF_t KF_Yaw, KF_Roll, KF_Pitch;

#endif
