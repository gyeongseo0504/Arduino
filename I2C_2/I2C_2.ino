#include <Wire.h>

union test
{
  int test1;
  byte mbyte[2];
};

void setup() 
{
  Wire.begin();
  Serial.begin(9600);
}

void loop() 
{
  union test n;

  Serial.print("mbyte[0] = ");
  while (!Serial.available()) {}
  n.mbyte[0] = Serial.parseInt();
  Serial.read();

  Serial.print("mbyte[1] = ");
  while (!Serial.available()) {}
  n.mbyte[1] = Serial.parseInt();
  Serial.read();
  Serial.println(n.test1);
  Wire.beginTransmission(1);
  Wire.write(n.mbyte[0]);
  Wire.write(n.mbyte[1]);
  Wire.endTransmission();
  delay(1000);
}
//Wire.write(1);
  //Wire.write(2);
  //Wire.write(3);
