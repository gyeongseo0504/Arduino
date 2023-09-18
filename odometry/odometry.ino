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

#include <MsTimer2.h>

#define wheel_track  0.15  //m 단위로 구할 것
#define RAD2DEG(x) ( x*180.0 / 3.14159 )
#define DEG2RAD(x) ( x*3.14159 / 180.0 )


const byte outPin = 13; // Output pin: digital pin 13(D13)
//const byte interruptPin1 = 2; // Interrupt pin: D2
//const byte interruptPin2 = 3; // Interrupt pin: D2
const byte encoder1_A = 2; // Interrupt pin: D2
const byte encoder1_B = 3; // Interrupt pin: D2
const byte encoder2_A = 18; // Interrupt pin: D2
const byte encoder2_B = 19; // Interrupt pin: D2

const byte resetPin = 5;
volatile byte state = 0;
unsigned long cnt1 = 0; // 추가
unsigned long cnt2 = 0; // 추가
unsigned long cnt1_old = 0; // 추가
unsigned long cnt2_old = 0; // 추가

//long cnt1 = 0; // 추가
//long cnt2 = 0; // 추가

const double pulse_to_distance_left = 0.2 / 512;
const double pulse_to_distance_right = 0.2 / 961;


const double odom_left  = 0;
const double odom_right = 0;

double yaw=0.0;
double yaw_degree=0.0;

struct Pose2D
{
  double x;
  double y;
  double theta;
}my_odem;

double heading(double x, double y)
{
  double head = atan2(y, x); // slope Y, Slope X
  return head;
}



void msTimer2_ISR()
{
  char msg[100] = {0x00,};
  double odom_left_delta  = 0.0;
  double odom_right_delta = 0.0;
  double odom_center_delta  = 0.0;
  long   delta_encoder_left = 0;
  long   delta_encoder_right = 0;
  double theta_delta = 0.0;
  double theta_delta_degree = 0.0;

  delta_encoder_left  = cnt1 - cnt1_old;
  delta_encoder_right = cnt2 - cnt2_old;

  //delta_encoder_left  = 100;
  //delta_encoder_right = -100;
  
  //sprintf(msg,"encoder delta : [%3d %3d]", delta_encoder_left, delta_encoder_right);
  //Serial.println(msg);
  Serial.print("delta_encoder_left = ");  Serial.print( delta_encoder_left);    Serial.print("    ");
  Serial.print("delta_encoder_right = "); Serial.println( delta_encoder_right); Serial.print("    ");
  
  odom_left_delta =  delta_encoder_left * pulse_to_distance_left;
  odom_right_delta = delta_encoder_right * pulse_to_distance_right;
  odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5; //곱하는게 더 빨라서 나누기는 너무 연산이 느림 

  Serial.print("odom_left_delta = ");     Serial.print(odom_left_delta);        Serial.print("    ");
  Serial.print("odom_right_delta = ");    Serial.println(odom_right_delta);     Serial.print("    ");

  theta_delta = heading(wheel_track, (odom_right_delta - odom_left_delta)); 
  Serial.print("delta theta raidan : ");  Serial.println(theta_delta);
  yaw           += theta_delta; 
  theta_delta_degree = RAD2DEG(theta_delta);
  Serial.print("delta theta degree : ");  Serial.println(theta_delta_degree);   Serial.println("");
  yaw_degree    += theta_delta_degree;
  Serial.print("yaw degree :");           Serial.println(yaw_degree);           Serial.println("");

  my_odem.x += odom_center_delta * sin(theta_delta);
  my_odem.y += odom_center_delta * cos(theta_delta);
  my_odem.theta += theta_delta;
  
  Serial.print("odem : ");
  Serial.print(my_odem.x);Serial.print(" ");
  Serial.print(my_odem.y);Serial.print(" ");
  Serial.println(my_odem.theta);
  
  /*Serial.print("delta_encoder_left = "); Serial.print( delta_encoder_left); Serial.print("    ");
  Serial.print("delta_encoder_right = "); Serial.println( delta_encoder_right); Serial.print("    ");

  Serial.print("odom_left_delta = "); Serial.print(odom_left_delta); Serial.print("    ");
  Serial.print("odom_right_delta = "); Serial.println(odom_right_delta); Serial.print("    ");

  Serial.print("wheel_track = ");  Serial.println(wheel_track); Serial.print("    ");

  Serial.print("theta_delta = "); Serial.println(theta_delta); Serial.print("    ");

  theta_delta_degree = DEG2RAD(theta_delta);
  Serial.print("theta_delta_degree = "); Serial.println(theta_delta_degree); Serial.print("    ");*/
  
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}
void setup()
{
  pinMode(outPin, OUTPUT); // Output mode
  //  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  //  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
  //  attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt
  my_odem.x = 0;   my_odem.y = 0;    my_odem.theta = 0;
  Serial.begin(115200);
  MsTimer2::set(100,msTimer2_ISR);
  MsTimer2::start();
}

/*
  void intfunc1() // Interrupt function
  {
  cnt1++;
  if (state == 0) // If D4 output is low
  {
    digitalWrite(outPin, HIGH);
    state = 1;
  }
  else
  {
    digitalWrite(outPin, LOW);
    state = 0;
  }
  }

  void intfunc2() // Interrupt function
  {
  cnt2++;
  if (state == 0) // If D4 output is low
  {
    digitalWrite(outPin, HIGH);
    state = 1;
  }
  else
  {
    digitalWrite(outPin, LOW);
    state = 0;
  }
  }

*/
void intfunc1()
{
  if (digitalRead(encoder1_A) == digitalRead(encoder1_B))
  {
    cnt1--;
  }
  else
  {
    cnt1++;
  }
}

void intfunc2()
{
  if (digitalRead(encoder2_B) == LOW)
  {
    cnt2--;
  }
  else
  {
    cnt2++;
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
void loop()
{
  //전진
  motor_A_control(HIGH,100);  //오른쪽 전진
  motor_B_control(LOW,100);  //왼쪽은 전진
  delay(1000);
  motor_A_control(HIGH,0);  //오른쪽 전진
  motor_B_control(LOW,0);  //왼쪽은 전진
  delay(1000);
  //회전
  const double odom_left = cnt1 * pulse_to_distance_left;
  const double odom_right = cnt2 * pulse_to_distance_right;

  delay(1000);
  
  
  /*Serial.print(cnt1); Serial.print("cnt1:   ");
  Serial.println(cnt2);
  Serial.print(odom_left); Serial.print("odom_left:   ");
  Serial.println(odom_right);
    
    
 Serial.print("cnt1: ");
  Serial.print(cnt1);
  Serial.print("   cnt2: ");
  Serial.println(cnt2);*/
}
