#define TRIG1 3 //초음파센서 1번 trig 핀 번호
#define ECHO1 4 //초음파센서 1번 trig 핀 번호

#define TRIG1 5 //초음파센서 1번 trig 핀 번호
#define ECHO1 6 //초음파센서 1번 trig 핀 번호

#define TRIG1 7 //초음파센서 1번 trig 핀 번호
#define ECHO1 8 //초음파센서 1번 trig 핀 번호



void setup() {
  // put your setup code here, to run once:
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1,  INPUT);
  Serial.begin(115200); //통신속도 15200로 정의

}

long sonar1(void) //초음파센서  1번 측정ㅍ 함수
{
  // put your main code here, to run repeatedly:
  long duration, distance;
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  duration = pulseIn(ECHO1, HIGH);
  distance = ((float)(340 * duration)/1000)/2;
  return distance;
}
void loop()
{ 
  //Serial.print("Duration:");
  //Serial.println(duration);
  Serial.print("Distance: ");
  Serial.println(sonar1());
  delay(500);
}
