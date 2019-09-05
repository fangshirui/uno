#include <Arduino.h>

int led = 13;
long count = 0;
void setup()
{
    // put your setup code here, to run once:
    pinMode(led, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    digitalWrite(led, HIGH);
    delay(1000);
    digitalWrite(led, LOW);
    delay(1000);

    count++;
    Serial.println(count);

    // put your main code here, to run repeatedly:
}