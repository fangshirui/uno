/********
FangShiRui �޸���2019.3
*********/
#include "MsTimer2.h" //��ʱ�жϿ�
#include "PinChangeInt.h" // �ⲿ�ж���չ��
#include <Arduino.h>
//���ò������̼���ʵ���ٶ�PID����
#include "KalmanFilter.h"
#include <BalanceCar.h>
//I2Cdev��MPU6050��PID_v1�����Ҫ���Ȱ�װ��Arduino ����ļ�����
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu; //ʵ����һ�� MPU6050 ���󣬶�������Ϊ mpu
BalanceCar balancecar; // ʵ����һ����Ϊbalancecar �Ķ���
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz; //˫�ֽ�����
//TB6612FNG����ģ������ź�
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

//�����Զ������

double kp = 38, ki = 0.0, kd = 0.58; //ƽ��pidֵ���ɵ�

float K1 = 0.05; // �Լ��ٶȼ�ȡֵ��Ȩ��,һ���˲��Ĳ���

float angle0 = 0.00; //��еƽ���

/////////////////////�������˲�����/////////////////////////////

float Q_angle = 0.001, Q_gyro = 0.005; //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
float R_angle = 0.5, C_0 = 1;

/////////////////////�������˲�����/////////////////////////////

float timeChange = 5; //�˲�������ʱ��������
float dt = timeChange * 0.001; //ע�⣺dt��ȡֵΪ�˲�������ʱ��

//////////////////�Ƕ�PD////////////////////
void angleout()
{
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x; //PD �ǶȻ�����,����.angle˵����kalmanfilter�����е����ԣ�
}
//////////////////�Ƕ�PD////////////////////

//////////////////////////////////////////////////////////
//////////////////�ж϶�ʱ 5ms��ʱ�ж�////////////////////
/////////////////////////////////////////////////////////
void inter()

{

    sei(); //����Ӧ���Ǵ����е��жϡ����Ϊ�жϵĳ�ʼ���������øú�������Ӱ��mpu�����ݻ�ȡ
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //IIC��ȡMPU6050�������� ax ay az gx gy gz
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); //�õ������������˲��ĸ�����angle���ͻ����ĸ����ǽ��ٶ�
    angleout(); //�õ��ǶȻ��Ŀ�����banlancecar.angleoutput

    balancecar.pwma(kalmanfilter.angle, kalmanfilter.angle6, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB); //С����PWM���
}
//////////////////////////////////////////////////////////
//////////////////�ж϶�ʱ 5ms��ʱ�ж�///////////////////
/////////////////////////////////////////////////////////

// ===    ��ʼ����     ===
void setup()
{
    // TB6612FNGN����ģ������źų�ʼ��

    pinMode(IN1M, OUTPUT); //���Ƶ��1�ķ���01Ϊ��ת��10Ϊ��ת
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT); //���Ƶ��2�ķ���01Ϊ��ת��10Ϊ��ת
    pinMode(IN4M, OUTPUT);
    pinMode(PWMA, OUTPUT); //����PWM
    pinMode(PWMB, OUTPUT); //�ҵ��PWM
    pinMode(STBY, OUTPUT); //TB6612FNGʹ��

    //��ʼ���������ģ��
    digitalWrite(IN1M, 0);
    digitalWrite(IN2M, 1);
    digitalWrite(IN3M, 1);
    digitalWrite(IN4M, 0);
    digitalWrite(STBY, 1);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    ;

    // ����I2C����
    Wire.begin(); //���� I2C ��������
    Serial.begin(9600); //�������ڣ����ò�����Ϊ 115200
    delay(1500);
    mpu.initialize(); //��ʼ��MPU6050
    delay(2);
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    //5ms��ʱ�ж�����  ʹ��timer2    ע�⣺ʹ��timer2���pin3 pin11��PWM�����Ӱ�죬��ΪPWMʹ�õ��Ƕ�ʱ������ռ�ձȣ�������ʹ��timer��ʱ��Ҫע��鿴��Ӧtimer��pin�ڡ�
    MsTimer2::set(5, inter); // ���ö�ʱ�жϣ�5msִ��һ��inter����
    MsTimer2::start(); //��ʱ�жϿ�ʼ��ʱ
}

////////////////////////////////////////turn//////////////////////////////////

// ===       ��ѭ��������       ===
void loop()
{
    attachInterrupt(0, );
}