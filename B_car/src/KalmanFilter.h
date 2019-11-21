
#ifndef KalmanFilter_h
#define KalmanFilter_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Kalman.h>

class KalmanFilter: private Kalman {
public:
    double angle1,angle6;
    double gyro_x,gyro_y;
    double m_angle, m_angle6;
    KalmanFilter();
    void update(double ax,double ay, double az,double gx,double gy, double gz);

private:
    double dt;

};
#endif
