#include <MsTimer2.h>
#include "sbus.h"

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////////////////// sbus ///////////
#define Sbus_Speed_max  200
#define Sbus_Speed_zero  1000
#define Sbus_Speed_min   1800

#define Sbus_str_max  1800
#define Sbus_str_zero  1040
#define Sbus_str_min   240

/* SBUS 객체, SBUS 읽기 */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS 객체, SBUS 쓰기 */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS 데이터 */
bfs::SbusData data;


struct MotorData
{
  int motor_speed;
  int str_position;
};
MotorData MD;

//int motor_speed;
//int str_position;
/////////////////////////// Steering_Control//////////////////
#define STEER_MOTOR_PWM              2
#define STEER_MOTOR_IN1              3
#define STEER_MOTOR_IN2              4

#define RIGHT_MOTOR_PWM              5
#define RIGHT_MOTOR_IN1              6
#define RIGHT_MOTOR_IN2              7

#define LEFT_MOTOR_PWM               8
#define LEFT_MOTOR_IN1               9
#define LEFT_MOTOR_IN2              10

#define STEERING_ANGLE_PIN          A0
#define STEERING_ANGLE_CONTROL_PIN  A2

#define NEURAL_ANGLE               448
#define LEFT_STEER_ANGLE           560
#define RIGHT_STEER_ANGLE          350

#define Max_Speed                  240

#define NO_OF_DATA                  15

int Steering_Angle;
int Steering_Angle_Control;

float Kp_Steer         = 3.0;
float Ki_Steer         = 0.0;
float Kd_Steer         = 5.0;
//3.5 7.5
//

float error            = 0.0;
float error_s          = 0.0;
float error_d          = 0.0;
float error_old        = 0.0;

float steering_control = 0.0;
float control_pwm      = 0.0;
float target_steering  = 0.0;
int target_angle       = 0;

float Data[NO_OF_DATA] = {0,};

void read_Potentiometer(void)
{
  Steering_Angle = analogRead(STEERING_ANGLE_PIN);
  Steering_Angle_Control = analogRead(STEERING_ANGLE_CONTROL_PIN);
}

void Ms2Timer_setup(void)
{
  MsTimer2::set(10, control_steering);
  MsTimer2::start();
}

////////////////////////////// SETUP ////////////////////////////////
void setup()
{
#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  pinMode(STEER_MOTOR_PWM, OUTPUT);
  pinMode(STEER_MOTOR_IN1, OUTPUT);
  pinMode(STEER_MOTOR_IN2, OUTPUT);

  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  pinMode(LEFT_MOTOR_PWM,  OUTPUT);
  pinMode(LEFT_MOTOR_IN1,  OUTPUT);
  pinMode(LEFT_MOTOR_IN2,  OUTPUT);

  //시리얼 통신이 확립될 때까지 기다리기 위함
  //디버깅 목적으로 시리얼 모니터를 사용할 때 중요
  while (!Serial) {}
  /* SBUS 통신 시작 */
  sbus_rx.Begin();
  sbus_tx.Begin();

  Ms2Timer_setup();

  Serial.begin(115200);
}

void steer_motor_control(int speed)
{
  if (speed >= 0)
  {
    if (speed >= Max_Speed)
    {
      speed = Max_Speed;
    }
    digitalWrite(STEER_MOTOR_IN1, LOW);
    digitalWrite(STEER_MOTOR_IN2, HIGH);
    analogWrite(STEER_MOTOR_PWM, speed);
  }
  else
  {
    if (speed <= -Max_Speed)
    {
      speed = -Max_Speed;
    }
    digitalWrite(STEER_MOTOR_IN1, HIGH);
    digitalWrite(STEER_MOTOR_IN2, LOW);
    analogWrite(STEER_MOTOR_PWM, -speed);
  }
}

void motor_control(int speed)
{
  if (speed >= 0)
  {
    if (speed >= Max_Speed)
    {
      speed = Max_Speed;
    }

    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_PWM, speed);

    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_PWM, speed);
  }
  else
  {
    if (speed <= -Max_Speed)
    {
      speed = -Max_Speed;
    }
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, -speed);

    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_PWM, -speed);
  }
}

void PID_Control(void)
{
  error = target_angle - Steering_Angle;
  error_d = error - error_old;

  steering_control = Kp_Steer * error + Kd_Steer * error_d;

  error_old = error;

  control_pwm = -steering_control;
  steer_motor_control((int)control_pwm);
}

void target_angle_limit(void)  //target_angle을 전역 변수로 쓸 경우
{
  if (target_angle < RIGHT_STEER_ANGLE)
  {
    target_angle = RIGHT_STEER_ANGLE;
  }
  else if (target_angle > LEFT_STEER_ANGLE)
  {
    target_angle = LEFT_STEER_ANGLE;
  }
  else
  {
    target_angle = target_angle;
  }
}
///////////////////////////////////////////////// sbus ///////////

void Sbus_motor_control()
{
  if (sbus_rx.Read())
  {
    data = sbus_rx.data();
    for (int8_t i = 0; i < 20; i++)
    {
      Serial.print(data.ch[i]);
      Serial.print("\t");
    }
    Serial.println("");

    sbus_tx.data(data);
    sbus_tx.Write();
    /*
        if (data.ch[1] >= Sbus_Speed_zero)
        {
          MD.motor_speed = map(data.ch[1], Sbus_Speed_zero, Sbus_Speed_min, 0, -200);
        }
        else
        {
          MD.motor_speed = map(data.ch[1], Sbus_Speed_max, Sbus_Speed_zero, 200, 0);
        }
    */
    if (data.ch[1] >= Sbus_Speed_zero + 30)
    {
      MD.motor_speed = map(data.ch[1], Sbus_Speed_zero, Sbus_Speed_min, 0, -50);
    }
    else if (data.ch[1] < Sbus_Speed_zero - 30)
    {
      MD.motor_speed = map(data.ch[1], Sbus_Speed_max, Sbus_Speed_zero, 50, 0);
    }
    else
    {
      MD.motor_speed = 0;
    }

    if (data.ch[0] > Sbus_str_zero)
    {
      MD.str_position = map(data.ch[0], Sbus_str_min, Sbus_str_zero, 560, 448);
    }
    else
    {
      MD.str_position = map(data.ch[0], Sbus_str_zero, Sbus_str_max, 448, 350);
    }

    //Serial.println(motor_speed);
    //Serial.println(motor_speed2);
    //Serial.println(str_position);
  }
}

void joy_contorl()
{
  Sbus_motor_control();
  if (data.ch[6] == 1800)
  {
    motor_control(MD.motor_speed);
    target_angle = MD.str_position;
  }
  else
  {
    motor_control(0);
  }
}

void Steering_Angle_Print(void)
{
  Serial.print("Steering_Angle: ");
  Serial.println(Steering_Angle);
  //Serial.print("Steering_Angle_Control: ");
  //Serial.println(Steering_Angle_Control);

  Serial.print("steering_control: ");
  Serial.println(steering_control);
  Serial.print("target_angle: ");
  Serial.println(target_angle);
  Serial.print("\n");
}

void control_steering(void)
{
  joy_contorl();
  read_Potentiometer();
  //target_angle_limit();
  PID_Control();
}

void loop()
{
  //Steering_Angle_Print();

  //steer_motor_control(0);
  //target_angle = 448;
  target_angle_limit();

  //motor_control(0);
}
