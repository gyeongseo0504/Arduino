/////////////////////////// Steering_Control//////////////////
#define MOTOR_DIR    2
#define MOTOR_PWM    3
#define STEERING_ANGLE A0
//#define NEURAL_ANGLE 512
#define LEFT_STEER_ANGLE 572
#define RIGHT_STEER_ANGLE 367

int Steering_Angle;
int NEURAL_ANGLE;

////////////////////////////// SETUP ////////////////////////////////
void setup()
{
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Serial.begin(115200);
}

void motor_control(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, speed);
  }
  else
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, -speed);
  }
}

void neural_angle(void)
{
  if (Steering_Angle >= 400 && Steering_Angle <= 600)
  {
    NEURAL_ANGLE = Steering_Angle;
  }
}

void steering_angle(void)
{
  neural_angle();
  Steering_Angle = analogRead(STEERING_ANGLE);

  if (Steering_Angle < NEURAL_ANGLE) //오른쪽
  {
    motor_control(200);
  }
  else if (Steering_Angle > NEURAL_ANGLE) //왼쪽
  {
    motor_control(-200);
  }
  else
  {
    motor_control(0);
  }
}

void target_angle(int target_angle)
{
  neural_angle();
  Steering_Angle = analogRead(STEERING_ANGLE);

  if (target_angle > NEURAL_ANGLE)
  {
    motor_control(200);
  }
  else if (target_angle < NEURAL_ANGLE)
  {
    motor_control(-200);
  }
  else
  {
    motor_control(0);
  }
}

void Steering_Angle_Print(void)
{
  Steering_Angle = analogRead(STEERING_ANGLE);

  Serial.print("Steering_Angle: ");
  Serial.println(Steering_Angle);
}

void loop()
{
  //target_angle(300);
  //motor_control(0);
  steering_angle();
  Steering_Angle_Print();
}
