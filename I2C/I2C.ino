#include <Wire.h>
#include <Servo.h>

Servo servoMotor;

union test
{
  int test1;
  byte mbyte[2];
};

void setup() 
{
  servoMotor.attach(8);
  Wire.begin(1);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void receiveEvent(int parameter)
{
  union test n;
  while(Wire.available() > 0)
  {
    for(int i = 0; i < 2; i++)
    {
      n.mbyte[i] = Wire.read();
    }
    //int a = Wire.read();
    Serial.print(n.test1);
    Serial.print(" ");
    servoMotor.write(90 + n.test1);
    
  }
}

void loop() 
{
}
