/********
FangShiRui 修改于2019.3
*********/
#include "MsTimer2.h"       //定时中断库
#include "PinChangeInt.h"   // 外部中断拓展库
#include <Arduino.h>
//利用测速码盘计数实现速度PID控制
#include "KalmanFilter.h"
#include <BalanceCar.h>
//I2Cdev、MPU6050和PID_v1类库需要事先安装在Arduino 类库文件夹下
#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050      mpu;          //实例化一个 MPU6050对象，对象名称为 mpu
BalanceCar   balancecar;   // 实例化一个名为balancecar 的对象
KalmanFilter kalmanfilter;
int16_t      ax, ay, az;   // x,y,z方向的加速度
int16_t      gx, gy, gz;   // x,y,z方向的角加速度，

//TB6612FNG驱动模块控制信号
#define IN1M 7    //右电机控制端口 IN1
#define IN2M 6    //右电机控制端口 IN2
#define IN3M 13   //左电机控制端口 IN1
#define IN4M 12   //左电机控制端口 IN2
#define PWMA 9    //右电机PWM端口
#define PWMB 10   //左电机PWM端口
#define STBY 8    //standby 端口，置零则电机停止

// 码盘中断端口号
#define PinA_left 2    // 自带的外部中断0 ，优先级很高，端口在D2
#define PinA_right 4   // 用户设定外部中断， 端口在D4

//* 声明自定义变量
double kp = 38, ki = 0.0, kd = 0.58;                        //平衡pid值，可调
double kp_speed = 3.5, ki_speed = 0.1058, kd_speed = 0.0;   // 速度pid值

//* 传感器数据
float K1     = 0.05;   // 对加速度计取值的权重,一阶滤波的参数
float angle0 = 0.00;   //机械平衡角

//* 卡尔曼滤波参数
float Q_angle = 0.001, Q_gyro = 0.005;   //角度数据置信度,角速度数据置信度
float R_angle = 0.5, C_0 = 1;

float timeChange = 5;                    //滤波法采样时间间隔毫秒
float dt         = timeChange * 0.001;   //注意：dt的取值为滤波器采样时间

//* 速度量
volatile long count_right = 0;   //右侧脉冲数量，一个周期归零，
volatile long count_left  = 0;   // 左侧脉冲数量，一个周期归零，
int           speedcc     = 0;   //速度环时间周期计数量，累积到8 则40ms周期

//* 脉冲计算相关参数
int rpluse = 0;   // 带正负号的当前周期右侧脉冲数量
int lpluse = 0;   // 带正负号的当前周期左侧脉冲数量
// int sumam;        // 历史所有周期，左右脉冲数量带正负号的总和

//* 角度PD
void angleout()
{
    //PD 角度环控制,angle说明是kalmanfilter对象中的属性，
    balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;
}

void countpulse();

//* 中断定时 5ms
void inter()
{

    //意义应该是打开所有的中断。理解为中断的初始化。若不用该函数，会影响mpu的数据获取
    sei();

    // 计算脉冲
    countpulse();

    //IIC获取MPU6050六轴数据 ax ay az gx gy gz
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //得到经过卡尔曼滤波的俯仰角angle，和基本的俯仰角角速度
    kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);

    //得到角度环的控制量banlancecar.angleoutput
    angleout();

    // 速度环
    speedcc++;
    if (speedcc >= 8) {
        balancecar.speedpiout(kp_speed, ki_speed, kd_speed, 0);
        speedcc = 0;
    }

    //小车总PWM输出
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

// ===    初始设置     ===
void setup()
{
    // TB6612FNGN驱动模块控制信号初始化

    pinMode(IN1M, OUTPUT);   //控制电机1的方向，01为正转，10为反转
    pinMode(IN2M, OUTPUT);
    pinMode(IN3M, OUTPUT);   //控制电机2的方向，01为正转，10为反转
    pinMode(IN4M, OUTPUT);
    pinMode(PWMA, OUTPUT);   //左电机PWM
    pinMode(PWMB, OUTPUT);   //右电机PWM
    pinMode(STBY, OUTPUT);   //TB6612FNG使能

    //初始化电机驱动模块
    digitalWrite(IN1M, 0);
    digitalWrite(IN2M, 1);
    digitalWrite(IN3M, 1);
    digitalWrite(IN4M, 0);
    digitalWrite(STBY, 1);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    // 测速码盘的设定
    pinMode(PinA_left, INPUT);
    pinMode(PinA_right, INPUT);

    // 加入I2C总线
    Wire.begin();         //加入 I2C 总线序列
    Serial.begin(9600);   //开启串口，设置波特率为 9600
    delay(1500);
    mpu.initialize();   //初始化MPU6050
    delay(2);
    balancecar.pwm1 = 0;
    balancecar.pwm2 = 0;
    //5ms定时中断设置  使用timer2    注意：使用timer2会对pin3 pin11的PWM输出有影响，因为PWM使用的是定时器控制占空比，所以在使用timer的时候要注意查看对应timer的pin口。
    MsTimer2::set(5, inter);   // 设置定时中断，5ms执行一次inter函数
    MsTimer2::start();         //定时中断开始计时
}

void loop()
{
    // 循环检测及叠加脉冲，测定小车车速
    attachInterrupt(0, code_left, CHANGE);                      // 使用自带0号中断
    attachPinChangeInterrupt(PinA_right, code_right, CHANGE);   // 使用自定义端口的中断
}

//* 脉冲计算
void countpulse()
{
    lpluse = count_left;
    rpluse = count_right;

    // 将当前周期脉冲数归零，进行下一周期的统计
    count_left  = 0;
    count_right = 0;

    if (balancecar.pwm1 < 0) {
        lpluse = -lpluse;
    }
    if (balancecar.pwm2 < 0) {
        rpluse = -rpluse;
    }

    // //提起判断
    // balancecar.stopr += rpluse;
    // balancecar.stopl += lpluse;

    //每5ms进入中断时，脉冲数叠加
    balancecar.pulseright += rpluse;
    balancecar.pulseleft += lpluse;
    // sumam = balancecar.pulseright + balancecar.pulseleft;
}