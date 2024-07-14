// 가변저항 => 0 ~ 1024

#define MOTOR_DIR    2
#define MOTOR_PWM    3
#define STEERING_ANGLE A0

int steer_angle;
int NEURAL_ANGLE;

int kp_steer = 0.3;
int kd_steer = 1;

int target;

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
  if (steer_angle >= 400 && steer_angle <= 600)
  {
    NEURAL_ANGLE = steer_angle;
  }
}

void Steering_Angle(void)
{
  steer_angle = analogRead(STEERING_ANGLE);
  neural_angle();
  
  static double steer_angle_old = 0;
  double steer_angle_d = 0;
  double steer_error = 0;
  
  steer_error = target - steer_angle;
  steer_angle_d = steer_error - steer_angle_old;
  
  double control_signal = (kp_steer * steer_error) + (kd_steer * steer_angle_d);   
  
  if (abs(steer_error) <= 3) 
  {
    motor_control(0);
  }
  else if (control_signal < NEURAL_ANGLE)
  {
    motor_control(200);
  }
  else if (control_signal > NEURAL_ANGLE) 
  {
    motor_control(-200);
  }

  steer_angle_old = steer_angle;
}

void Steering_Angle_Print(void)
{
  steer_angle = analogRead(STEERING_ANGLE);
  
  Serial.print("Steering_Angle: ");
  Serial.println(steer_angle);  
}

void loop()
{
  target = 400; // 목표 각도를 설정

  Steering_Angle();
  Steering_Angle_Print();
}
