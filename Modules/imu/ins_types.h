#ifndef INS_TYPES_H
#define INS_TYPES_H

typedef struct
{
    float q[4];
    float Gyro[3];
    float Accel[3];
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} attitude_t;

#endif
