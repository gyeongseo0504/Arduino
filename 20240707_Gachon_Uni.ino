#include <MsTimer2.h>

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////////// Steering_Control//////////////////
#define MOTOR_DIR               2
#define MOTOR_PWM               3
#define STEERING_ANGLE         A0
#define STEERING_ANGLE_CONTROL A2

#define NEURAL_ANGLE        465
#define NEURAL_ANGLE_OFFSET 3

#define LEFT_STEER_ANGLE    550
#define RIGHT_STEER_ANGLE   370

#define Target_Angle_Error 3

#define Max_Speed 200

int Steering_Angle;
int Steering_Angle_old;
int Steering_Angle_Control;

int neural_angle;

float Kp_Steer = 3.0;
float Ki_Steer = 0.0;
float Kd_Steer = 1.0;

float error            = 0;
float error_s          = 0;
float error_d          = 0;
float error_old        = 0;
float steering_control = 0;
float control_pwm;
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

  interrupt_setup();

  Serial.begin(115200);
}

void read_Potentiometer(void)
{
  Steering_Angle = analogRead(STEERING_ANGLE);
  Steering_Angle_Control = analogRead(STEERING_ANGLE_CONTROL);
}

void interrupt_setup(void)
{
  MsTimer2::set(10, read_Potentiometer); // Set the timer to call read_analog_inputs every 10ms
  MsTimer2::start();
}

void motor_control(int speed)
{
  if (speed >= 0)
  {
    if (speed >= Max_Speed)
    {
      speed = Max_Speed;
    }

    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, speed);
  }
  else
  {
    if (speed <= -Max_Speed)
    {
      speed = -Max_Speed;
    }
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, -speed);
  }
}

void Neural_Angle(void)
{
  if (Steering_Angle >= (NEURAL_ANGLE - NEURAL_ANGLE_OFFSET) && Steering_Angle <= (NEURAL_ANGLE + NEURAL_ANGLE_OFFSET))
  {
    neural_angle = Steering_Angle;
  }
}

float PID_Control(int target_angle)
{
  Neural_Angle();

  error = target_angle - Steering_Angle;
  error_d = error - error_old;

  steering_control = Kp_Steer * error + Kd_Steer * error_d;
  Serial.print("  steering_control: ");
  Serial.println(steering_control);
  error_old = error;

  return -steering_control;
}

void target_angle(int target_angle)
{
  if (target_angle < RIGHT_STEER_ANGLE)
  {
    target_angle = RIGHT_STEER_ANGLE;
  }
  else if (target_angle > LEFT_STEER_ANGLE)
  {
    target_angle = LEFT_STEER_ANGLE;
  }

  Neural_Angle();

  control_pwm = PID_Control(target_angle);
  motor_control((int)control_pwm);
}

void Steering_Angle_Print(void)
{
  Serial.print("Steering_Angle: ");
  Serial.print(Steering_Angle);
  Serial.print("  Steering_Angle_Control: ");
  Serial.println(Steering_Angle_Control);
}
void loop()
{
  Steering_Angle_Print();

  //target_angle(530);
  target_angle(Steering_Angle_Control);
}
