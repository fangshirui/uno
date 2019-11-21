#include "Kalman.h"
#include "KalmanFilter.h"
#include "MPU6050.h"
#include "MsTimer2.h"
#include <Arduino.h>
#include <wire.h>

MPU6050      mpu;
KalmanFilter ka;
Kalman       ka2;
double       dt = 0.005;
double       K1 = 0.05;

int16_t ax, ay, az, gx, gy, gz;

String s = "";
char   angle[7], angle6[7], m_angle[7], m_angle6[7], ka2_angle_char[7];
float  ka2_angle = 0;

void inter()
{
    sei();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ka.Angletest(ax, ay, az, gx, gy, dt, K1);
    ka2_angle = ka2.getAngle(ka.m_angle, ka.Gyro_x, dt);
}

void setup()
{
    Wire.begin();
    Serial.begin(38400);   //如果使用蓝牙模块，需要在蓝牙模块手动设置为38400 使用AT命令
    delay(1000);
    mpu.initialize();
    delay(200);

    MsTimer2::set(5, inter);
    MsTimer2::start();
}

void loop()
{
    dtostrf(ka.angle, 3, 2, angle);
    dtostrf(ka.angle6, 3, 2, angle6);
    dtostrf(ka.m_angle, 3, 2, m_angle);
    dtostrf(ka.m_angle6, 3, 2, m_angle6);
    dtostrf(ka2_angle, 3, 2, ka2_angle_char);

    String str_angle    = angle;
    String str_angle6   = angle6;
    String str_m_angle  = m_angle;
    String str_m_angle6 = m_angle6;

    s = str_angle + "," + str_angle6 + "," + str_m_angle + "," + str_m_angle6 + "," + (String)ka2_angle_char;
    Serial.println(s);
    // Serial.print("ok");
    delay(5);
}