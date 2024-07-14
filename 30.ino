#include <Servo.h>

union test
{
   int num;
   byte angle[2];
};

void setup() 
{
  Serial.begin(115200);
}

void loop()
{
  union test n;
  
  n.num = -30;
  for(int i = 0; i < 2; i++)
  {
    Serial.print("mbyte[");
    Serial.print(i);
    Serial.print("] = ");
    Serial.println(n.angle[i]);
  }

  Serial.println(" ");
  Serial.println(n.num);

  delay(500);
}
