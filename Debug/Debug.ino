#include <MsTimer2.h>

#if ARDUINO >= 100
const int led_pin = LED_BUILTIN;
#else
const int led_pin = 13;
#endif

const int debug_pin1 = 33;  
const int debug_pin2 = 34;

void serialprint() 
{
  digitalWrite(debug_pin2, HIGH);
  Serial.println("debug");
  digitalWrite(debug_pin2, LOW);
}

void flash() 
{
  digitalWrite(debug_pin1, HIGH);
  serialprint();  // 시리얼 통신과 함께 신호를 확인할 핀
  digitalWrite(debug_pin1, LOW);
}

void setup() 
{
  pinMode(led_pin, OUTPUT);
  pinMode(debug_pin1, OUTPUT);
  pinMode(debug_pin2, OUTPUT);
  Serial.begin(115200);
  MsTimer2::set(20, flash); // 20ms 주기로 flash 함수 호출
  MsTimer2::start();
}

void loop() {
}
