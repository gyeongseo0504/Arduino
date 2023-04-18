////////////////////////// sonar ////////////////////////
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE      400 //mm 단위 
#define WALL_GAP_DISTANCE_HALF 200 //mm 단위
#define MOTOR_PWM_OFFSET 10
 
#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2  // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3  // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 4  // 초음파 센서 2번 Trig 핀 번호 왼쪽
#define ECHO2 5  // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 6  // 초음파 센서 3번 Trig 핀 번호 오른쪽
#define ECHO3 7  // 초음파 센서 3번 Echo 핀 번호

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};

  float front_sonar   = 0.0;
  float left_sonar    = 0.0;
  float right_sonar   = 0.0;

///////////////////////// L298N //////////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

///////////////////////// Maze Status ////////////////////
int maze_status = 0;

void setup() 
{
  //put your setup code here, to run once:
 /*
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
*/
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
 
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  Serial.begin(115200); // 통신속도를 115200으로 정의함 
}

long sonar_front(void)  //초음파 센서 1번 측정 함수
{
  long duration1, distance1; 
  digitalWrite(TRIG1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG1, LOW);
  duration1 = pulseIn(ECHO1, HIGH);
  distance1 = ( (float) (340*duration1)/1000)/2;
  return distance1;
}

long sonar2(void)  //초음파 센서 2번 측정 함수
{
  long duration2, distance2; 
  digitalWrite(TRIG2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG2, LOW);
  duration2 = pulseIn(ECHO2, HIGH);
  distance2 = ( (float) (340*duration2)/1000)/2;
  return distance2;
}

long sonar3(void)  //초음파 센서 3번 측정 함수
{
  long duration3, distance3; 
  digitalWrite(TRIG3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG3, LOW);
  duration3 = pulseIn(ECHO3, HIGH);
  distance3 = ( (float) (340*duration3)/1000)/2;
  return distance3;
}

void motor_A_contro1(int direction_a, int motor_speed_a) //모터 A의 방향(direction)과 속도 (speed)제어
{
   if(direction_a == HIGH)
   {
     digitalWrite(IN1, HIGH);               // 모터의 방향 제어
     digitalWrite(IN2, LOW);
     analogWrite(ENA,motor_speed_a);        // 모터의 속도 제어
   }
   else
   {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
     analogWrite(ENA,motor_speed_a);
   }

}

  
void motor_B_contro1(int direction_b, int motor_speed_b) //모터 B의 방향(direction)과 속도 (speed)제어
{
   if(direction_b == HIGH)
   {
     digitalWrite(IN3, HIGH);               // 모터의 방향 제어
     digitalWrite(IN4, LOW);
     analogWrite(ENB,motor_speed_b);        // 모터의 속도 제어
   }
   else
   {
     digitalWrite(IN3, LOW);
     digitalWrite(IN4, HIGH);
     analogWrite(ENB,motor_speed_b);

   }

}


void check_maze_status(void)
{
   if( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) )
 {  //3면을 다 막았을때
     maze_status = 4;
     Serial.println("maze_status = 4");
 }
  else if( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) )
 {
     maze_status = 1;
     Serial.println("maze_status = 1");
 }
  else  if( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF) )
 {
     maze_status = 2;
     Serial.println("maze_status = 2");
 }
  else  if( (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF) )
 {
     maze_status = 3;
     Serial.println("maze_status = 3");
 }
  else
 {
    maze_status = 0;
    Serial.println("maze_status = 0");
 }

}

// 먼저 left_pwm = 0;  right_pwm = 100; 으로 해서 왼쪽 오른쪽 방향 찾기

void wall_collision_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.7;   // 나중에 조정해야할 값
  int pwm_control = 0;
  int right_pwm = 0;
  int left_pwm = 0;
  error = right_sonar - left_sonar;
  error = Kp * error;
  
  right_pwm = base_speed - error;
  left_pwm  = base_speed + error;
  
  if(right_pwm <= 0 ) right_pwm = 0;
  if(left_pwm  <= 0 )  left_pwm = 0;

  if(right_pwm >= 225 ) right_pwm = 225;
  if(left_pwm  >= 225 )  left_pwm = 225;
  
  
  motor_A_contro1(LOW,right_pwm);  //오른쪽은 전진
  motor_B_contro1(HIGH,left_pwm);  //왼쪽은 전진
}

void loop()
{
  //put your main code here, to run repeatedly:

 
  front_sonar = sonar[Front].ping_cm()*10;       // 전방 센서 측정
  left_sonar  = sonar[Left].ping_cm()*10;        // 좌측 센서 측정
  right_sonar = sonar[Right].ping_cm()*10;       // 우측 센서 츨측정
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE;        // 0.0 출력이 최대값이므로
  if(left_sonar  == 0.0)  left_sonar  = MAX_DISTANCE;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE;

  Serial.print("L:");Serial.print(left_sonar);    Serial.print("  ");
  Serial.print("F:");Serial.print(front_sonar);   Serial.print("  ");
  Serial.print("R:");Serial.println(right_sonar);    

  check_maze_status();

 /*
  if(maze_status == 4)
  {
    Serial.println("Rotate stop");
    motor_A_contro1(HIGH,0);  //오른쪽 정지
    motor_B_contro1(LOW,0);  //왼쪽은 정지
    delay(500);                //일정한 시간 동안 정지
    
     //180도 회전을 한다
    Serial.println("Rotate CCW");
    motor_A_contro1(HIGH,118);  //오른쪽은 전진
    motor_B_contro1(HIGH,120);  //왼쪽은 전진
    delay(1000);

    Serial.println("Rotate stop");
    motor_A_contro1(HIGH,0);  //오른쪽 정지
    motor_B_contro1(LOW,0);  //왼쪽은 정지
    delay(500);                //일정한 시간 동안 정지
  }
 else if(maze_status == 3)
  {
    Serial.println("Rotate stop");
    motor_A_contro1(HIGH,0);  //오른쪽 정지
    motor_B_contro1(LOW,0);  //왼쪽은 정지
    delay(500);                //일정한 시간 동안 정지
    
    //앞면 오른면만 있을 때 왼쪽
    Serial.println("Rotate CCW");
    motor_A_contro1(HIGH,82);  //오른쪽은 전진
    motor_B_contro1(HIGH,79);  //왼쪽은 전진
    delay(940);

    Serial.println("Rotate stop");
    motor_A_contro1(HIGH,0);  //오른쪽 정지
    motor_B_contro1(LOW,0);  //왼쪽은 정지
    delay(500);                //일정한 시간 동안 정지
  }
   else if(maze_status == 2)
  {
    Serial.println("Rotate stop");
    motor_A_contro1(HIGH,0);  //오른쪽 정지
    motor_B_contro1(LOW,0);  //왼쪽은 정지
    delay(500);                //일정한 시간 동안 정지
    
    //앞면 왼쪽면만 있을 때 오른쪽
    Serial.println("Rotate CCW");
    motor_A_contro1(LOW,95);  //오른쪽은 전진
    motor_B_contro1(LOW,90);  //왼쪽은 전진
    delay(750); 

    Serial.println("Rotate stop");
    motor_A_contro1(HIGH,0);  //오른쪽 정지
    motor_B_contro1(LOW,0);  //왼쪽은 정지
    delay(500);                //일정한 시간 동안 정지

    Serial.println("Rotate CCW");
    motor_A_contro1(LOW,85);  //오른쪽은 전진
    motor_B_contro1(LOW,90);  //왼쪽은 전진
    delay(800);
  }
  
  else*/if(maze_status == 1)
  {
    //좌우 벽만 있을 때 전진
    Serial.println("run straight");
    wall_collision_avoid(100);
  }
  else
  {
  
  }
  
}
