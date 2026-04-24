#include "main.h"
#include "gray_track.h"
#include "pid.h"
#include "mpu6050.h"
#include "math.h"

//int last_state = 0; // 全局或静态变量，记录上次偏离方向：1为偏左，2为偏右

//void track() 
//{
//    // 1. 直行：黑线在正中间 (11011)
//    if((O3 == 0) && (O2 != 0) && (O4 != 0))
//    {
//        pid_set_tar_speed(40, 40);   // 基准速度20直行
//        last_state = 0; 
//    }
//    
//    // --- 左转逻辑 ---
//    
//    // 2. 微偏左 (10011)
//    else if((O2 == 0) && (O3 == 0)) 
//    {
//        pid_set_tar_speed(18, 10);   // 右轮稍快，左轮减速
//        last_state = 1;
//    }
//    // 3. 偏左 (10111)
//    else if((O2 == 0)) 
//    {
//        pid_set_tar_speed(15, 5);    
//        last_state = 1;
//    }
//    // 4. 急偏左 (00111)
//    else if((O1 == 0) && (O2 == 0)) 
//    {
//        pid_set_tar_speed(15, -5);   // 左轮反转！强行拉回车头
//        last_state = 1;
//    }
//    // 5. 极限最左 (01111)
//    else if(O1 == 0) 
//    {
//        pid_set_tar_speed(15, -10);  // 强力反转，应对极窄弯道
//        last_state = 1;
//    }
//    
//    // --- 右转逻辑 ---
//    
//    // 6. 微偏右 (11001)
//    else if((O3 == 0) && (O4 == 0)) 
//    {
//        pid_set_tar_speed(10, 18);
//        last_state = 2;
//    }
//    // 7. 偏右 (11101)
//    else if(O4 == 0) 
//    {
//        pid_set_tar_speed(5, 15);
//        last_state = 2;
//    }
//    // 8. 急偏右 (11100)
//    else if((O4 == 0) && (O5 == 0)) 
//    {
//        pid_set_tar_speed(-5, 15);   // 右轮反转
//        last_state = 2;
//    }
//    // 9. 极限最右 (11110)
//    else if(O5 == 0) 
//    {
//        pid_set_tar_speed(-10, 15);  // 强力反转
//        last_state = 2;
//    }
//    
//    // 10. 异常/丢失状态 (11111)
//    else if((O1 && O2 && O3 && O4 && O5) == 1)
//    {
//        // 如果全白，根据最后一次看到线的方向，原地旋转找线
//        if(last_state == 1) pid_set_tar_speed(12, -12); // 刚才偏左，现在向左原地自旋
//        else if(last_state == 2) pid_set_tar_speed(-12, 12); // 刚才偏右，向右原地自旋
//        else pid_set_tar_speed(0, 0); // 未知情况，停止或慢速滑行
//    }
//    
//    // 11. 特殊状态：全黑 (00000) 十字路口
//    else 
//    {
//        pid_set_tar_speed(15, 15); // 默认直行穿过
//    }
//}

int last_err = 0;
float Kp = 30;
float Kd = 1;
float Kpp = 2;
float Kdd = 2;

int track_error(void)
{
    int sum = 0;
    int cnt = 0;

    if (O1 == 0) { sum -= 1.5; cnt++; }
    if (O2 == 0) { sum -= 0.3; cnt++; }
    if (O4 == 0) { sum += 0.3; cnt++; }
    if (O5 == 0) { sum += 1.5; cnt++; }

    if (cnt == 0)
    {
        return last_err > 0 ? 3 : -3;
    }

    return sum / cnt;
}

void track(void)
{
    double err = track_error();
    double derr = err - last_err;
    last_err = err;

    float out = Kp * err + Kd * derr + Kpp * (err*fabs(err)) + Kdd * (float)(gz - gyro_zero_z)/16.4f;

    int base = 50;
    int right = base - (int)out;
    int left  = base + (int)out;

    if (right < 0) right = 0;
    if (left < 0) left = 0;
    if (right > 70) right = 70;
    if (left > 70) left = 70;

    pid_set_tar_speed(right, left);
}

