#include <MsTimer2.h>

unsigned long time1;
unsigned long time2;

void flash()
{
  static boolean output = HIGH;

  digitalWrite(3, output);
  output = !output;
}

void setup()
{
  pinMode(3, OUTPUT);
  Serial.begin(115200);

  MsTimer2::set(20, flash); // 500ms period
  MsTimer2::start();
}

void loop()
{
  /*
    time1 = micros();
    digitalWrite(3, HIGH);
    time2 = micros();
    Serial.print(time2 - time1);
    digitalWrite(3, LOW);
  */
  /*
    time1 = micros();
    digitalWrite(3, HIGH);
    while ( (micros() - time1) <= 19 )
    {

    }
    digitalWrite(3, LOW);
  */
}
