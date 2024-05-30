#ifndef KALMAN_H
#define KALMAN_H

extern double dt; // Measurement period

typedef struct {
    double x;
    double y;
    double vx;
    double vy;
    double est[4];

    double PData[16];
    double QData[16];
    double RData[4];
    double HData[8];
    double AData[16];
    double KGData[8];
    double IData[16];
} KalmanFilter;

void kalman_init(KalmanFilter* kf);
void kalmanFilter(KalmanFilter* kf, double *measured);

#endif
