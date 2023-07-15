#include <MsTimer2.h>
//////// Motor Drive ///////////
#define check 2
#define MOTOR1_PWM 5 //PWMA
#define MOTOR1_EN 4 //DIRA
int motor1_pwm = 0;
int f_speed    = 0;

///////// Encoder Pin //////////
const int EncoderAPin = 2;
const int EncoderBPin = 3;

volatile int counter  = 0;
volatile int encoderB;

///////// Interrupt time ///////
unsigned long enc_time;
unsigned long enc_time_old;
unsigned long enc_time_diff;

////////  Encoder PID /////////
signed long encoder1count =      0;

signed long encoder1_error =     0;

signed long encoder1_error_d =   0;

signed long encoder1_target =    0;

signed long encoder1_error_old = 0;

signed long encoder1_error_sum = 0;

// PID 게인 값은 조정 필요함
float Kp_motor = 1.8;
float Kd_motor = 2.0;
float Ki_motor = 0.18;

void motor1_control(int motor1_pwm)
{
  if (motor1_pwm > 0)     // cw
  {
    digitalWrite(MOTOR1_EN, HIGH);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // ccw
  {
    digitalWrite(MOTOR1_EN, LOW);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else                     // stop
  {
    digitalWrite(MOTOR1_EN, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}

void motor_PID_control(void)
{
  encoder1count = counter;
  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_sum += encoder1_error;
  encoder1_error_d = encoder1_error - encoder1_error_old;
  encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;
  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;

  motor1_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  motor1_pwm = (motor1_pwm >=  255) ?  255 : motor1_pwm;
  motor1_pwm = (motor1_pwm <= -255) ? -255 : motor1_pwm;

  if (fabs(encoder1_error) <= 2)
  {
    encoder1_error_sum = 0;
  }
  else
  {
    motor1_control(motor1_pwm);
  }
  encoder1_error_old = encoder1_error;
}

void Encoder() 
{
  enc_time = millis();
  enc_time_diff = enc_time - enc_time_old;

  encoderB = digitalRead(EncoderBPin);

  if (encoderB == LOW) // 정회전
  {    
    counter++;
  }
  else                 // 역회전
  {                   
    counter--;
  }
  enc_time_old = enc_time;
}

void control_callback() 
{
  enc_time = millis();
  enc_time_diff = enc_time - enc_time_old;

  if (counter < encoder1_target) 
  {
    motor_PID_control();
  }
  else {
    motor1_control(0);
  }
  enc_time_old = enc_time;
}

void setup() 
{
  Serial.begin(115200);

  pinMode(EncoderAPin, INPUT_PULLUP);
  pinMode(EncoderBPin, INPUT_PULLUP);

  pinMode(check, OUTPUT);
  pinMode(MOTOR1_EN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(EncoderAPin), Encoder, RISING);

  MsTimer2::set(50, control_callback); // 50ms(20hz)마다 호출한다
  MsTimer2::start();
}

void loop1() 
{
  encoder1_target = 410; // target encoder 값으로 조정 필요함
  Serial.print("Encoder 1 : ");    Serial.println(encoder1count);
  Serial.print("Encoder error 1 : ");    Serial.println(encoder1_error);
}
void loop()
{
  encoder1_target = 4100; // target encoder 값으로 조정 필요함
  Serial.print("A: 1");
  Serial.print("   B: ");
  Serial.print(encoderB);
  Serial.print("   Counter: ");
  Serial.print(counter);
  Serial.println();
  Serial.print("enc_time_diff: ");
  Serial.print(enc_time_diff);
  Serial.println();
  Serial.print("motor1_pwm: ");
  Serial.println(motor1_pwm);
}
