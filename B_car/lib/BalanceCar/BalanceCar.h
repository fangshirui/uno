/********
FangShiRui �޸���2019.3
*********/

#ifndef BalanceCar_h // ��ֹ���ε�������ļ�
#define BalanceCar_h

/////////////////��Ӹ����⺯��//////////////////////////////////////////////////

#include <Arduino.h>
///////////////////////////////////////////////////////////////////////////////

class BalanceCar
{
public:
    void pwma(float angle, float angle5, int Pin1, int Pin2, int Pin3, int Pin4, int PinPWMA, int PinPWMB);
    // int pulseright = 0;
    // int pulseleft = 0;
    // int posture=0;
    // int stopl = 0;
    // int stopr = 0;
    double angleoutput = 0, pwm1 = 0, pwm2 = 0;

private:
};
#endif
