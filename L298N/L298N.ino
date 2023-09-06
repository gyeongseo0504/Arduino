#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

void setup()
{
  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENB, OUTPUT);

  Serial.begin(115200);
}

void motor_R_control(int motor_speed_R) // 모터 A의 속도(speed)제어
{
  if (motor_speed_R >= 0)
  {
    digitalWrite(IN3, HIGH);         //모터의 방향 제어
    digitalWrite(IN4, LOW);
    if (motor_speed_R >= 255) motor_speed_R = 255;
    analogWrite(ENB, motor_speed_R); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    if (motor_speed_R <= -255) motor_speed_R = -255;
    analogWrite(ENB, -motor_speed_R);
  }
}


void motor_L_control(int motor_speed_L) // 모터 A의 속도(speed)제어
{
  if (motor_speed_L >= 0)
  {
    digitalWrite(IN1, LOW);         //모터의 방향 제어
    digitalWrite(IN2, HIGH);
    if (motor_speed_L >= 255) motor_speed_L = 255;
    analogWrite(ENA, motor_speed_L); //모터의 속도 제어
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    if (motor_speed_L <= -255) motor_speed_L = -255;
    analogWrite(ENA, -motor_speed_L);
  }
}


void loop()
{
  motor_R_control(120);
  motor_L_control(120);
}
