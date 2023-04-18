//////////////// sonar ////////////////////////
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.

#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3 // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 4 // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 5 // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 6 // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 7 // 초음파 센서 3번 Echo 핀 번호
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};
//////////////// L298 /////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

float front_sonar = 0.0;
float left_sonar  = 0.0;
float right_sonar = 0.0;
///////////  Maze status  /////////////////
int maze_status = 0;

void setup()
{

  Serial.begin(115200); // 통신속도를 115200으로 정의함

}
long sonar_front(void) // 초음파 센서 1번 측정 함수
{
  long duration, distance;
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  duration = pulseIn(ECHO1, HIGH);
  distance = ( (float) (340*duration)/1000)/2;
  return distance;
}

long sonar2(void) // 초음파 센서 2번 측정 함수
{
  long duration, distance;
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  duration = pulseIn(ECHO2, HIGH);
  distance = ( (float) (340*duration)/1000)/2;
  return distance;
}

long sonar3(void) // 초음파 센서 3번 측정 함수
{
  long duration, distance;
  digitalWrite(TRIG3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG3, LOW);
  duration = pulseIn(ECHO3, HIGH);
  distance = ( (float) (340*duration)/1000)/2;
  return distance;
}
void check_maze_status(void)
{
  if( (left_sonar>= 0)&&(left_sonar <= 200)&&(front_sonar>= 0)&&(front_sonar <= 200)&&(right_sonar>= 0)&&(right_sonar <= 200)  )
  {  // 3면이 다 막힌 경우
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if( (left_sonar>= 0)&&(left_sonar <= 200)&&(right_sonar>= 0)&&(right_sonar <= 200)   )
  {
    maze_status = 1;
    Serial.println("maze_status =1");
  }
  else if( (left_sonar>= 0)&&(left_sonar <= 200)&&(front_sonar>= 0)&&(front_sonar <= 200)   )
  {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if( (right_sonar>= 0)&&(right_sonar <= 200)&&(front_sonar>= 0)&&(front_sonar <= 200)   )
  {
    maze_status = 3;
    Serial.println("maze_status = 3");  
  }
  
  else
  {
   maze_status = 0;
   Serial.println("maze_status = 0;");
  }
  
}
void loop() 
{
  // put your main code here, to run repeatedly:
   
  //Serial.print("Duratin: ");
  //Serial.println(duration);

  front_sonar = sonar[Front].ping_cm()*10;    // 전방 센서 측정
  left_sonar = sonar[Left ].ping_cm()*10;     // 좌측 센서 측정
  right_sonar = sonar[Right].ping_cm()*15;    // 우측 센서 측정
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10;    // 0.0 출력이 최대값이므로
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE*10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE*10;

  Serial.print("L:");Serial.print(left_sonar);  Serial.print(" ");
  Serial.print("F:");Serial.print(front_sonar);  Serial.print(" ");
  Serial.print("R:");Serial.println (right_sonar);

  check_maze_status();
  delay(1000);
  if (maze_status == 4)
  {
    Serial.println("Rotate CCW");
    motor_A_control(LOW,100);  //오른쪽 전진
    motor_B_control(HIGH ,100);  //왼쪽은 후진
    delay(1000);           

  }
  else if( maze_status == 1)
  {
    Serial.println("Rotate CCW");
    motor_A_control(LOW,100);  //오른쪽 전진
    motor_B_control(LOW,100);  //왼쪽은 전진
  }
    else if( maze_status == 2)
  {
    Serial.println("Rotate CCW");
    motor_A_control(HIGH,100);  //오른쪽 전진
    motor_B_control(LOW,100);  //왼쪽은 전진
  }
    else if( maze_status == 3)
  {
    Serial.println("Rotate CCW");
    motor_A_control(LOW,100);  //오른쪽 전진
    motor_B_control(HIGH,100);  //왼쪽은 전진
  } 
}
void motor_A_control(int direction_a, int motor_speed_a) //모터 A의 방향 (direction)과 속도(speed)제어
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1,HIGH);           //모터의 방향 제어
    digitalWrite(IN2,LOW);
    analogWrite(ENA,motor_speed_a);   //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,motor_speed_a);
  }
}
void motor_B_control(int direction_b, int motor_speed_b) //모터 B의 방향 (direction)과 속도(speed)제어
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3,LOW);           //모터의 방향 제어
    digitalWrite(IN4,HIGH);
    analogWrite(ENB,motor_speed_b);  //모터의 속도 제어
  }
  else
  {
    
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
    analogWrite(ENB,motor_speed_b);
  }
}
