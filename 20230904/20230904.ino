#include <MsTimer2.h>

const int signalPin = 1;

void setup() 
{
  pinMode(signalPin, OUTPUT);

  MsTimer2::set(1, timer);
  MsTimer2::start();
}

void loop() 
{
  
}

void timer() 
{
  static boolean outputState = LOW;

  if (outputState == LOW) 
  {
    digitalWrite(signalPin, HIGH);
    outputState = HIGH;
  } 
  else 
  {
    digitalWrite(signalPin, LOW);
    outputState = LOW;
  }
}
