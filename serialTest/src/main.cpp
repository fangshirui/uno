#include <Arduino.h>

int     led   = 13;
long    count = 0;
int     incomingByte;
int     num1            = 0;
boolean startBit        = false;
boolean newLineReceived = false;
String  inputString     = "";

int  mark = 2;
void setup()
{
    // put your setup code here, to run once:
    pinMode(led, OUTPUT);
    Serial.begin(9600);
}

void loop()
{
    if (newLineReceived) {
        if (inputString[1] == '1') {
            mark = 1;
        } else if (inputString[1] == '0') {
            mark = 0;
        } else {
            mark = 2;
        }
    }

    if (mark == 0) {
        digitalWrite(led, HIGH);
        delay(10);
        digitalWrite(led, LOW);
        delay(10);
        Serial.print("$22,44#");
    } else if (mark == 1) {
        digitalWrite(led, HIGH);
        delay(10);
        digitalWrite(led, LOW);
        delay(10);
        Serial.print("$22,76#");
    } else {
        digitalWrite(led, LOW);
        Serial.print("$43,63#");
        delay(20);
    }

    inputString     = "";
    newLineReceived = false;
}

void serialEvent()
{
    while (Serial.available()) {        // 放在缓存区
        incomingByte = Serial.read();   // 一个字节的读
        if (incomingByte == '$') {
            num1     = 0;
            startBit = true;
        }

        if (startBit == true) {
            num1++;
            inputString += (char)incomingByte;
        }

        if (startBit == true && incomingByte == '#') {
            newLineReceived = true;
            startBit        = false;
        }

        if (num1 > 3) {
            num1            = 0;
            startBit        = false;
            newLineReceived = false;
            inputString     = "";
        }
    }
}