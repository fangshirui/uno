/********
FangShiRui �޸���2019.3
*********/
#include "MsTimer2.h"       //��ʱ�жϿ�
#include "PinChangeInt.h"   // �ⲿ�ж���չ��
#include <Arduino.h>
//���ò������̼���ʵ���ٶ�PID����
#include "KalmanFilter.h"
#include <BalanceCar.h>
//I2Cdev��MPU6050��PID_v1�����Ҫ���Ȱ�װ��Arduino ����ļ�����
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050      mpu;          //ʵ����һ�� MPU6050���󣬶�������Ϊ mpu
BalanceCar   balancecar;   // ʵ����һ����Ϊbalancecar �Ķ���
KalmanFilter kalmanfilter;
int16_t      ax, ay, az;   // x,y,z����ļ��ٶ�
int16_t      gx, gy, gz;   // x,y,z����ĽǼ��ٶȣ�

//TB6612FNG����ģ������ź�
#define IN1M 7    //�ҵ�����ƶ˿� IN1
#define IN2M 6    //�ҵ�����ƶ˿� IN2
#define IN3M 13   //�������ƶ˿� IN1
#define IN4M 12   //�������ƶ˿� IN2
#define PWMA 9    //�ҵ��PWM�˿�
#define PWMB 10   //����PWM�˿�
#define STBY 8    //standby �˿ڣ���������ֹͣ

// �����ж϶˿ں�
#define PinA_left 2    // �Դ����ⲿ�ж�0 �����ȼ��ܸߣ��˿���D2
#define PinA_right 4   // �û��趨�ⲿ�жϣ� �˿���D4

//* �����Զ������
double kp = 38, ki = 0.0, kd = 0.58;                        //ƽ��pidֵ���ɵ�
double kp_speed = 3.5, ki_speed = 0.1058, kd_speed = 0.0;   // �ٶ�pidֵ

//* ����������
float K1     = 0.05;   // �Լ��ٶȼ�ȡֵ��Ȩ��,һ���˲��Ĳ���
float angle0 = 0.00;   //��еƽ���

//* �������˲�����
float Q_angle = 0.001, Q_gyro = 0.005;   //�Ƕ��������Ŷ�,���ٶ��������Ŷ�
float R_angle = 0.5, C_0 = 1;

float timeChange = 5;                    //�˲�������ʱ��������
float dt         = timeChange * 0.001;   //ע�⣺dt��ȡֵΪ�˲�������ʱ��

//* �ٶ���
volatile long count_right = 0;   //�Ҳ�����������һ�����ڹ��㣬
volatile long count_left  = 0;   // �������������һ�����ڹ��㣬
int           speedcc     = 0;   //�ٶȻ�ʱ�����ڼ��������ۻ���8 ��40ms����

//* ���������ز���
int rpluse = 0;   // �������ŵĵ�ǰ�����Ҳ���������
int lpluse = 0;   // �������ŵĵ�ǰ���������������
// int sumam;        // ��ʷ�������ڣ��������������������ŵ��ܺ�

//* �Ƕ�PD
void angleout()
{
    //PD �ǶȻ�����,angle˵����kalmanfilter�����е����ԣ�
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;
}

void countpulse();

//* �ж϶�ʱ 5ms
void inter()
{

    //����Ӧ���Ǵ����е��жϡ����Ϊ�жϵĳ�ʼ���������øú�������Ӱ��mpu�����ݻ�ȡ
    sei();

    // ��������
    countpulse();

    //IIC��ȡMPU6050�������� ax ay az gx gy gz
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //�õ������������˲��ĸ�����angle���ͻ����ĸ����ǽ��ٶ�
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    //�õ��ǶȻ��Ŀ�����banlancecar.angleoutput
    angleout();

    // �ٶȻ�
    speedcc++;
    if (speedcc >= 8) {
        balancecar.speedpiout(kp_speed, ki_speed, kd_speed, 0);
        speedcc = 0;
    }

    //С����PWM���
    balancecar.pwma(kalmanfilter.angle, kalmanfilter.angle6, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);
}

void code_left()
{
    count_left++;
}

void code_right()
{
    count_right++;
}

// ===    ��ʼ����     ===
void setup()
{
    // TB6612FNGN����ģ������źų�ʼ��

    pinMode(IN1M, OUTPUT);   //���Ƶ��1�ķ���01Ϊ��ת��10Ϊ��ת
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT);   //���Ƶ��2�ķ���01Ϊ��ת��10Ϊ��ת
    pinMode(IN4M, OUTPUT);
    pinMode(PWMA, OUTPUT);   //����PWM
    pinMode(PWMB, OUTPUT);   //�ҵ��PWM
    pinMode(STBY, OUTPUT);   //TB6612FNGʹ��

    //��ʼ���������ģ��
    digitalWrite(IN1M, 0);
    digitalWrite(IN2M, 1);
    digitalWrite(IN3M, 1);
    digitalWrite(IN4M, 0);
    digitalWrite(STBY, 1);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    // �������̵��趨
    pinMode(PinA_left, INPUT);
    pinMode(PinA_right, INPUT);

    // ����I2C����
    Wire.begin();         //���� I2C ��������
    Serial.begin(9600);   //�������ڣ����ò�����Ϊ 9600
    delay(1500);
    mpu.initialize();   //��ʼ��MPU6050
    delay(2);
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    //5ms��ʱ�ж�����  ʹ��timer2    ע�⣺ʹ��timer2���pin3 pin11��PWM�����Ӱ�죬��ΪPWMʹ�õ��Ƕ�ʱ������ռ�ձȣ�������ʹ��timer��ʱ��Ҫע��鿴��Ӧtimer��pin�ڡ�
    MsTimer2::set(5, inter);   // ���ö�ʱ�жϣ�5msִ��һ��inter����
    MsTimer2::start();         //��ʱ�жϿ�ʼ��ʱ
}

void loop()
{
    // ѭ����⼰�������壬�ⶨС������
    attachInterrupt(0, code_left, CHANGE);                      // ʹ���Դ�0���ж�
    attachPinChangeInterrupt(PinA_right, code_right, CHANGE);   // ʹ���Զ���˿ڵ��ж�
}

//* �������
void countpulse()
{
    lpluse = count_left;
    rpluse = count_right;

    // ����ǰ�������������㣬������һ���ڵ�ͳ��
    count_left  = 0;
    count_right = 0;

    if (balancecar.pwm1 < 0) {
        lpluse = -lpluse;
    }
    if (balancecar.pwm2 < 0) {
        rpluse = -rpluse;
    }

    // //�����ж�
    // balancecar.stopr += rpluse;
    // balancecar.stopl += lpluse;

    //ÿ5ms�����ж�ʱ������������
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    // sumam = balancecar.pulseright + balancecar.pulseleft;
}