#include <Servo.h>

// 서보 객체 생성
Servo myServo;

// 서보 핀 번호
const int servoPin = 9;

// 원하는 각도 (예: 90도)
int desiredAngle = 90;

void setup() {
  // 서보 핀을 서보 객체에 연결
  myServo.attach(servoPin);
  // 서보 모터를 원하는 각도로 이동
  myServo.write(desiredAngle);
}

void loop() {
  // 메인 루프에서는 아무 작업도 하지 않음
}
