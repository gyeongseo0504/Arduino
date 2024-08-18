#include <MsTimer2.h>
#include "sbus.h"

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////////// Sonar  //////////////////
#include <NewPing.h>

#define SONAR_NUM 4       // 초음파 센서 개수를 4개로 변경
#define MAX_DISTANCE 2000 // Maximum distance (in cm) to ping.

unsigned long previousMillis = 0.0;
const long interval          = 200;

union
{
  float data;
  byte bytedata[4];
} Left_Front_Sonar, Left_Rear_Sonar, Right_Front_Sonar, Right_Rear_Sonar;

NewPing sonar[SONAR_NUM] =
{
  NewPing(30, 31, MAX_DISTANCE), // 왼쪽 전방
  NewPing(32, 33, MAX_DISTANCE), // 왼쪽 후방
  NewPing(34, 35, MAX_DISTANCE), // 오른쪽 전방
  NewPing(36, 37, MAX_DISTANCE), // 오른쪽 후방
};

float car_angle = 0.0;

float Kp_Wall = 1.0;
float Ki_Wall = 0.0;
float Kd_Wall = 1.0;

float error_wall             = 0.0;
float error_wall_s           = 0.0;
float error_wall_d           = 0.0;
float error_wall_old         = 0.0;

/////////////////////////// Motor  //////////////////
#define encodPinA1   2
#define encodPinB1   3
#define MOTOR_DIR    4
#define MOTOR_PWM    5

#define Max_Speed   250

/////////////////////////// Steering  //////////////////
#include <Servo.h>

#define SERVO_PIN 3

#define P_LEFT_STEER_ANGLE    -50
#define P_RIGHT_STEER_ANGLE    50
#define P_NEURAL_ANGLE         90
#define P_NEURAL_ANGLE_offset  -3

#define NEURAL_ANGLE                 0
#define LEFT_STEER_ANGLE           -25
#define RIGHT_STEER_ANGLE           25

#define Steer_Max_Speed            240

int Steering_Angle = 0;

Servo Steeringservo;

/////////////////////////// Ms2Timer  //////////////////

void Ms2Timer_setup(void)
{
  //MsTimer2::set(20, control_steering);
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

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Steeringservo.attach(SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);
  
  Ms2Timer_setup();

  Serial.begin(115200);
}

//////////////////////////////////// MOTOR ////////////////////
void motor_control(int speed)
{
  if (speed >= 0) 
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, speed);
  }
  else 
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, -speed);
  }
}

int steering_control()
{
  if (Steering_Angle <= LEFT_STEER_ANGLE)  
  {
    Steering_Angle = LEFT_STEER_ANGLE;
  }
  if (Steering_Angle >= RIGHT_STEER_ANGLE) 
  {
    Steering_Angle = RIGHT_STEER_ANGLE;
  }

  Steeringservo.write(Steering_Angle + NEURAL_ANGLE);
}

void read_ultrasonic_sensor(void)
{
  Left_Front_Sonar.data   = sonar[0].ping_cm() * 10.0;
  Left_Rear_Sonar.data    = sonar[1].ping_cm() * 10.0;
  Right_Front_Sonar.data  = sonar[2].ping_cm() * 10.0;
  Right_Rear_Sonar.data   = sonar[3].ping_cm() * 10.0;

  if (Left_Front_Sonar.data == 0)
  {
    Left_Front_Sonar.data = MAX_DISTANCE;
  }
  if (Left_Rear_Sonar.data == 0)
  {
    Left_Rear_Sonar.data = MAX_DISTANCE;
  }
  if (Right_Front_Sonar.data == 0)
  {
    Right_Front_Sonar.data = MAX_DISTANCE;
  }
  if (Right_Rear_Sonar.data == 0)
  {
    Right_Rear_Sonar.data = MAX_DISTANCE;
  }
}

void calculate_car_angle(void)
{
  float Left_Angle  = atan2(Left_Front_Sonar.data - Left_Rear_Sonar.data, 100.0) * 180.0 / PI;
  float Right_Angle = atan2(Right_Front_Sonar.data - Right_Rear_Sonar.data, 100.0) * 180.0 / PI;

  car_angle = (Left_Angle + Right_Angle) / 2.0;
}

void ultrasonic_sensor_serial_print(void)
{
  read_ultrasonic_sensor();
  calculate_car_angle();

  Serial.print("Left_Front_Sonar : ");
  Serial.println(Left_Front_Sonar.data);
  Serial.print("Left_Rear_Sonar : ");
  Serial.println(Left_Rear_Sonar.data);

  Serial.print("Right_Front_Sonar : ");
  Serial.println(Right_Front_Sonar.data);
  Serial.print(" Right_Rear_Sonar: ");
  Serial.println(Right_Rear_Sonar.data);

  Serial.print("Angle : ");
  Serial.print(car_angle);
  Serial.println(" degrees");
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    read_ultrasonic_sensor();
    calculate_car_angle();
    ultrasonic_sensor_serial_print();
  }
}
