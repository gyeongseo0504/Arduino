//////////////// sonar ////////////////////////
#include <NewPing.h>

#define SONAR_NUM 3      
#define MAX_DISTANCE 150
#define WALL_GAP_DISTANCE 400      // mm 단위
#define WALL_GAP_DISTANCE_HALP 200 // mm 단위
#define MOTOR_PWM_OFFSET 10

#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3 // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 4 // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 5 // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 6 // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 7 // 초음파 센서 3번 Echo 핀 번호
  NewPing sonar[SONAR_NUM] = {  
  NewPing(TRIG1, ECHO1, MAX_DISTANCE),
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};
//////////////////////////// L298 //////////////////////////////
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
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
 
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  Serial.begin(115200); // 통신속도를 115200으로 정의함
}

void check_maze_status(void)
{
  if( (left_sonar>= 0)&&(left_sonar <= WALL_GAP_DISTANCE_HALP)&&(front_sonar>= 0)&&(front_sonar <= WALL_GAP_DISTANCE_HALP)&&(right_sonar>= 0)&&(right_sonar <= WALL_GAP_DISTANCE_HALP)  )
  {  //3면이 다막혔을때
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if( (left_sonar>= 0)&&(left_sonar <= WALL_GAP_DISTANCE_HALP)&&(right_sonar>= 0)&&(right_sonar <= WALL_GAP_DISTANCE_HALP)&&(right_sonar >= WALL_GAP_DISTANCE_HALP)   )
  {
    maze_status = 1;
    Serial.println("maze_status =1");
  }
  else if( (left_sonar>= 0)&&(left_sonar <= WALL_GAP_DISTANCE_HALP)&&(front_sonar>= 0)&&(front_sonar <= WALL_GAP_DISTANCE_HALP)   )
  {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if( (right_sonar>= 0)&&(right_sonar <= WALL_GAP_DISTANCE_HALP)&&(front_sonar>= 0)&&(front_sonar <= WALL_GAP_DISTANCE_HALP) )
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


//먼저 left_pwm = 0; right_pwm = 100; 으로 해서 왼쪽 오른쪽 방향찾기

void wall_collinion_avoid(int base_speed)
{
  float error = 0.0;
  float Kp = 0.7;  //나중에 조정해야 할 값
  int pwm_control = 0;
  int right_pwm = 0;
  int left_pwm = 0;
  error = (right_sonar - left_sonar);
  error = Kp * error;

  right_pwm = base_speed - error;
  left_pwm = base_speed + error;

 if (right_pwm <=0 ) right_pwm = 0;
 if (left_pwm <=0 ) left_pwm = 0;

 if (right_pwm <=225 ) right_pwm = 225;
 if (left_pwm <=225 ) left_pwm = 225;
  right_pwm = 0;
  left_pwm = 100;
  motor_A_control(HIGH,right_pwm);   //오른쪽 전진
  motor_B_control(HIGH,left_pwm);    //왼쪽 전진
}
void loop()
{
  front_sonar = sonar[Front].ping_cm()*10;    // 전방 센서 측정
  left_sonar = sonar[Left ].ping_cm()*10;     // 좌측 센서 측정
  right_sonar = sonar[Right].ping_cm()*10;    // 우측 센서 측정
 
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10;    // 0.0 출력이 최대값이므로
  if(left_sonar == 0.0) left_sonar = MAX_DISTANCE*10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE*10;

  Serial.print("L:");Serial.print(left_sonar);  Serial.print(" ");
  Serial.print("F:");Serial.print(front_sonar);  Serial.print(" ");
  Serial.print("R:");Serial.println (right_sonar);

  check_maze_status();

  if(maze_status == 4)
  {
    Serial.println("Rotare CCW");
    motor_A_control(HIGH,0);  
    motor_B_control(HIGH,0);    
    delay(1000);
    motor_A_control(HIGH,100);   //오른쪽 전진
    motor_B_control(LOW,100);    //왼쪽 후진
    delay(1160);
    motor_A_control(HIGH,0);  
    motor_B_control(HIGH,0);    
    delay(1000);
   
  }

  else if(maze_status == 2)
  {
    Serial.println("Rotare CCW");
    motor_A_control(HIGH,0);  
    motor_B_control(HIGH,0);    
    delay(1000);
    motor_A_control(LOW,100);   //오른쪽 전진
    motor_B_control(HIGH,100);    //왼쪽 후진
    delay(570);
    motor_A_control(HIGH,000);  
    motor_B_control(HIGH,000);    
    delay(1000);
    }
    else if(maze_status == 3)
  {
    Serial.println("Rotare CCW");
    motor_A_control(HIGH,0);  
    motor_B_control(HIGH,0);    
    delay(1000);
    motor_A_control(HIGH,100);   //오른쪽 전진
    motor_B_control(LOW,100);    //왼쪽 후진
    delay(570);
    motor_A_control(HIGH,0);  
    motor_B_control(HIGH,0);    
    delay(1000);
    }
    else if(maze_status == 1)
    {
    Serial.println("run stratight");
    wall_collinion_avoid(100);
    }
    else
    {
   
    }
     
  }
  void motor_A_control(int direction_a, int motor_speed_a)   // 모터 A의 방향(direction)과 속도(speed) 제어
{
  if(direction_a == HIGH)
  {
    digitalWrite(IN1,HIGH);            // 모터의 방향 제어
    digitalWrite(IN2,LOW);
    analogWrite(ENA,motor_speed_a);    // 모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
    analogWrite(ENA,motor_speed_a);
  }
}

void motor_B_control(int direction_b, int motor_speed_b)   // 모터 B의 방향(direction)과 속도(speed) 제어
{
  if(direction_b == HIGH)
  {
    digitalWrite(IN3,HIGH);            // 모터의 방향 제어
    digitalWrite(IN4,LOW);
    analogWrite(ENB,motor_speed_b);    // 모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
    analogWrite(ENB,motor_speed_b);
  }
}
 
