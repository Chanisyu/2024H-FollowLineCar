#include "main.h"
#include "gray_track.h"
#include "pid.h"
#include "mpu6050.h"
#include "math.h"

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

