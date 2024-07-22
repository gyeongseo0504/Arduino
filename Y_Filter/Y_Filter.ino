#include <MsTimer2.h>

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

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

#define NEURAL_ANGLE               465
#define LEFT_STEER_ANGLE           550
#define RIGHT_STEER_ANGLE          370

#define Max_Speed                  240

#define NO_OF_DATA                  15

int Steering_Angle;
int Steering_Angle_Control;
float Filter_Steering_Angle;

float Kp_Steer         = 3.5;
float Ki_Steer         = 0.0;
float Kd_Steer         = 7.5;

float error            = 0.0;
float error_s          = 0.0;
float error_d          = 0.0;
float error_old        = 0.0;

float steering_control = 0.0;
float control_pwm      = 0.0;
float target_steering  = 0.0;
int target_angle       = 0;

float data[NO_OF_DATA] = {0,};

float recursive_moving_average(float ad_value)
{
  static float avg = 0.0;

  for (int i = 0; i < NO_OF_DATA - 1; i++)
  {
    data[i] = data[i + 1];
    //Serial.print(data[i]);
    //Serial.print("  ");
  }
  data[NO_OF_DATA - 1] = ad_value;
  //Serial.println(data[NO_OF_DATA - 1]);
  //Serial.print("Average_old2: "); Serial.println(avg);

  avg = avg + ((data[NO_OF_DATA - 1] - data[0]) / (float)NO_OF_DATA);

  return avg;
}

void read_Potentiometer(void)
{
  Steering_Angle = analogRead(STEERING_ANGLE_PIN);
  Filter_Steering_Angle = recursive_moving_average(Steering_Angle);
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
  error = target_angle - Filter_Steering_Angle;
  error_d = error - error_old;

  steering_control = Kp_Steer * error + Kd_Steer * error_d;

  error_old = error;

  control_pwm = -steering_control;
  steer_motor_control((int)control_pwm);
}

void target_angle_limit(void)
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
  Serial.print("Filter_Steering_Angle: "); // 필터링된 조향각 출력
  Serial.println(Filter_Steering_Angle);
  
  Serial.print("\n");
}

void control_steering(void)
{
  read_Potentiometer();
  //target_angle_limit();
  PID_Control();
}

void loop()
{
  Steering_Angle_Print();
  
  //steer_motor_control(0);
  target_angle = 465;
  target_angle_limit();
  
  motor_control(0);
}
