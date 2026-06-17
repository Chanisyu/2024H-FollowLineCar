/**
 * @file    filter.c
 * @brief   姿态角互补滤波和一维卡尔曼滤波。
 *
 * 文件结构：
 *   1. Mahony_Filter()   - 按固定权重融合陀螺仪积分角和观测角
 *   2. Kalman_Filter()   - 用二状态卡尔曼滤波估计角度和陀螺仪零偏
 */

#include "filter.h"

// 互补滤波权重：越接近 1 越信任陀螺仪，响应更快但长期漂移更明显。
#define alpha  0.95238

// 卡尔曼参数：Q 越大越相信模型变化，R 越大越不信任加速度计/磁力计观测。
KF_t KF_Yaw={
  0.001,        // Q_angle：角度过程噪声，增大后响应更快但更容易抖。
  0.003,        // Q_bias：零偏过程噪声，增大后零漂跟踪更快但估计更不稳。
  0.5,          // R：观测噪声，增大后输出更平滑但跟随观测更慢。
  {{1,0},{0,1}},
  0.01          // dt：滤波周期 10ms，对应 MPU6050/定时器的 100Hz 数据节拍。
};

KF_t KF_Roll={
  0.001,
  0.003,
  0.5,
  {{1,0},{0,1}},
  0.01
};

KF_t KF_Pitch={
  0.001,
  0.003,
  0.5,
  {{1,0},{0,1}},
  0.01
};

float Mahony_Filter(float gyro,float acc)
{
  // gyro 响应快但会漂，acc/mag 不会长期漂但运动时噪声大，两者按固定比例折中。
  return (alpha*gyro+(1-alpha)*acc);
}

// ================================================================
// 1. 二状态卡尔曼滤波
//
//    状态量：
//      Angle      - 当前角度估计值，单位为度。
//      Gyro_bias  - 陀螺仪零偏估计值，单位与输入角速度一致。
//    输入：
//      obsValue   - 加速度计/磁力计观测角，低频可靠。
//      ut         - 陀螺仪角速度，高频响应快。
// ================================================================
float Kalman_Filter(KF_t *kf,float obsValue,float ut)
{
  // ------- 1. 预测角度 -------
  kf->Angle=kf->Angle+(ut-kf->Gyro_bias)*kf->dt;
  kf->Gyro_bias=kf->Gyro_bias;

  // ------- 2. 预测误差协方差 -------
  kf->P[0][0]=kf->P[0][0]-(kf->P[0][1]+kf->P[1][0])*kf->dt+kf->P[1][1]*kf->dt*kf->dt+kf->Q_angle;
  kf->P[0][1]=kf->P[0][1]-kf->P[1][1]*kf->dt;
  kf->P[1][0]=kf->P[1][0]-kf->P[1][1]*kf->dt;
  kf->P[1][1]=kf->P[1][1]+kf->Q_bias;

  // ------- 3. 计算卡尔曼增益 -------
  kf->K1=kf->P[0][0]/(kf->P[0][0]+kf->R);
  kf->K2=kf->P[1][0]/(kf->P[0][0]+kf->R);

  // ------- 4. 用观测值修正估计 -------
  kf->Angle=kf->Angle+kf->K1*(obsValue-kf->Angle);
  kf->Gyro_bias=kf->Gyro_bias+kf->K2*(obsValue-kf->Angle);

  // ------- 5. 收缩误差协方差 -------
  kf->P[0][0]=(1-kf->K1)*kf->P[0][0];
  kf->P[0][1]=(1-kf->K1)*kf->P[0][1];
  kf->P[1][0]=kf->P[1][0]-kf->P[0][0]*kf->K2;
  kf->P[1][1]=kf->P[1][1]-kf->P[0][1]*kf->K2;

  return kf->Angle;
}
