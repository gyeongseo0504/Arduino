#include <MsTimer2.h>                 // Timer를 실행하기 위한 라이브러리 추가  

#define m_2_pulse     348.             // 1m 당 pulse 수  확인 해야 함
#define pulse_2_m     1./348.         // pulse 당 m  확인 해야 함
#define vel_2_pulse   m_2_pulse/20.  // 100Hz 제어 주기에서 속도와 Δpulse 변환 값
#define NEURAL_ANGLE 4


signed long encoder1_target = 0;
int Steering_Angle = NEURAL_ANGLE;
unsigned char buf[30];
unsigned long Speed_CH;
unsigned long Steer_CH;
unsigned long Mode_CH;
int rc_mode,steer_data;
float speed_data;
int cnt = 0;
float target_velocity1 = 0.0;
float target_velocity2 = 0.0;

union
{
  float data ;
  char  bytedata[4];
    
} m_car_speed_float;

union
{
    short data ;
    char  bytedata[2];
    
} m_car_angle_int16;

union
{
    signed long data ;
    char  bytedata[4];
    
} m_car_encoder_long;

///////////////////////////////// 조종기 관련  /////////////////////////////////////
#include "SBUS.h"
SBUS x8r(Serial2);
uint16_t channels[16];
bool failSafe;
bool lostFrame;
int SPEED=0;
int Steering_angle=0;
int mode=0;

void control(){
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){
    x8r.write(&channels[0]);
  }

  for(int i=0; i<16; i++){
    Serial.print(channels[i]); Serial.print("  ");
    }
  Serial.println(" ");
  
  if(channels[0] != 0 && channels[14] == 992){//수신기가 작동하고 있을시
    
   if(channels[0] >= 800 && channels[0]<= 1200) SPEED=0; //속도제어
   else if(channels[0] < 800) {
    SPEED = ((974-channels[0])/4)*0.75;
    SPEED *= -1;
    SPEED /= 2;
   }
   else if(channels[0] > 1200){
    SPEED = (channels[0]-1200)/3;
    SPEED /= 2;
   }

   
   Steering_angle = -1*map(channels[1],172,1811,-15,15)+5;  //조향 제어

   if(200 >= channels[5])    mode = 1;
   else if(1200 <= channels[5]) mode = -1;
   else mode = 0;

//////////////////////////////컨트롤러 모드1/////////////////////////////////////////////
  if(mode == -1){
    motor_control(SPEED,SPEED);
    Steering_Angle = Steering_angle;
    clearEncoderCount(1); 
    clearEncoderCount(2);
    target_velocity1 = 0;
    encoder1_target = 0;
    //target_velocity1 = 0;
    }
//////////////////////////////컨트롤러 모드1/////////////////////////////////////////////
  else if(mode == 0){
    motor_control(SPEED,SPEED);
    clearEncoderCount(1); 
    clearEncoderCount(2);
    target_velocity1 = 0;
    encoder1_target = 0;
    //target_velocity1 = 0;
    }
  }
  else{ // 수신기 없는 경우 이 경우는 그냥 없이 하거나 아님 자율주행만 되거나 하도록 함
    SPEED=0;
    Steering_angle=0;
    }
  //Serial.println(SPEED);
  //Serial.println(mode);
    }

///////////////////////////////// Motor Drive  /////////////////////////////////////
#define MOTOR1_PWM 2
#define MOTOR1_ENA 3
#define MOTOR1_ENB 4

#define MOTOR2_PWM 5
#define MOTOR2_ENA 6
#define MOTOR2_ENB 7
int f_speed = 0, r_speed = 0;
int front_motor_pwm = 0;
int rear_motor_pwm = 0;

void front_motor_control(int motor1_pwm)
{
  if (motor1_pwm > 0) // forward
  {
    digitalWrite(MOTOR1_ENA, HIGH);
    digitalWrite(MOTOR1_ENB, LOW);
    analogWrite(MOTOR1_PWM, motor1_pwm);
  }
  else if (motor1_pwm < 0) // backward
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, HIGH);
    analogWrite(MOTOR1_PWM, -motor1_pwm);
  }
  else
  {
    digitalWrite(MOTOR1_ENA, LOW);
    digitalWrite(MOTOR1_ENB, LOW);
    digitalWrite(MOTOR1_PWM, 0);
  }
}

void rear_motor_control(int motor2_pwm)
{
   if (motor2_pwm > 0) // forward
  {
    digitalWrite(MOTOR2_ENA, HIGH);
    digitalWrite(MOTOR2_ENB, LOW);
    analogWrite(MOTOR2_PWM, motor2_pwm);
  }
  else if (motor2_pwm < 0) // backward
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, HIGH);
    analogWrite(MOTOR2_PWM, -motor2_pwm);
  }
  else
  {
    digitalWrite(MOTOR2_ENA, LOW);
    digitalWrite(MOTOR2_ENB, LOW);
    digitalWrite(MOTOR2_PWM, 0);
  }
}


void motor_control(int front_speed, int rear_speed)
{
  front_motor_control(front_speed);
  rear_motor_control(rear_speed);  
}

///////////////////////////////// Encoder  /////////////////////////////////////

#include <SPI.h>
#define ENC1_ADD 22
#define ENC2_ADD 23

signed long encoder1count = 0;
signed long encoder2count = 0;

signed long encoder1_error = 0;
signed long encoder2_error = 0;

signed long encoder1_error_d = 0;
signed long encoder2_error_d = 0;

signed long encoder2_target = 0;

signed long encoder1_error_old = 0; 
signed long encoder2_error_old = 0; 

signed long encoder1_error_sum = 0; 
signed long encoder2_error_sum = 0; 


void initEncoders() {
  
  // Set slave selects as outputs
  pinMode(ENC1_ADD, OUTPUT);
  pinMode(ENC2_ADD, OUTPUT); 
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(ENC1_ADD,HIGH);
  digitalWrite(ENC2_ADD,HIGH);
 
  SPI.begin();
  
  // Initialize encoder 1
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC1_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC1_ADD,HIGH);       // Terminate SPI conversation 

  // Initialize encoder 2
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(ENC2_ADD,LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(ENC2_ADD,HIGH);       // Terminate SPI conversation 
}

long readEncoder(int encoder_no) 
{  
  // Initialize temporary variables for SPI read
  unsigned int count_1, count_2, count_3, count_4;
  long count_value;   
  
  digitalWrite(ENC1_ADD + encoder_no-1,LOW);      // Begin SPI conversation
   // digitalWrite(ENC4_ADD,LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  //digitalWrite(ENC4_ADD,HIGH);      // Begin SPI conversation
// Calculate encoder count
  count_value= ((long)count_1<<24) + ((long)count_2<<16) + ((long)count_3<<8 ) + (long)count_4;
  
  return count_value;
}

void clearEncoderCount(int encoder_no) {    
  // Set encoder1's data register to 0
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
  
  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder1's current data register to center
  digitalWrite(ENC1_ADD+encoder_no-1,LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(ENC1_ADD+encoder_no-1,HIGH);     // Terminate SPI conversation 
}

/////////////////////////////////////// motor PID 제어  ////////////////////////////////////////////


float Kp_motor = 1.0;
float Kd_motor = 6.0;
float Ki_motor = 0.0;


void front_motor_PID_control(void)
{
  encoder1count = readEncoder(1);
  //Serial.println(encoder1count);
  encoder1_error = encoder1_target - encoder1count;
  encoder1_error_sum += encoder1_error;

  encoder1_error_d = encoder1_error - encoder1_error_old;
  encoder1_error_sum = (encoder1_error_sum >=  100) ?  100 : encoder1_error_sum;
  encoder1_error_sum = (encoder1_error_sum <= -100) ? -100 : encoder1_error_sum;
  
  front_motor_pwm = Kp_motor * encoder1_error + Kd_motor * encoder1_error_d + Ki_motor * encoder1_error_sum;
  front_motor_pwm = (front_motor_pwm >=  255) ?  255 : front_motor_pwm;
  front_motor_pwm = (front_motor_pwm <= -255) ? -255 : front_motor_pwm;
  
  if (fabs(encoder1_error) <= 1.0)
  {
    encoder1_error_sum = 0;
    clearEncoderCount(1); 
    clearEncoderCount(2);
    encoder1_target = encoder2_target = 0;
  }
  else 
  {
    motor_control(front_motor_pwm,front_motor_pwm);    
  }  
  encoder1_error_old = encoder1_error; 
}

///////////////////////////////////////  Steering PID 제어 /////////////////////////////////////////////

#define Steering_Sensor A15  // Analog input pin that the potentiometer is attached to
#define No_Calibration_Point 11
#define LEFT_STEER_ANGLE  20
#define RIGHT_STEER_ANGLE  -16
#define MOTOR3_PWM 8
#define MOTOR3_ENA 9
#define MOTOR3_ENB 10
#define AD_MIN 200
#define AD_MAX 800
#define alpha 0.37
struct 
{
  double X[No_Calibration_Point];
  double Y[No_Calibration_Point];
}cal_data;

float Kp = 12.0;
float Ki = 0.01;
float Kd = 0.2; //PID 상수 설정, 실험에 따라 정해야 함 중요!
double Setpoint, Input, Output; //PID 제어 변수
double error, error_old;
double error_s, error_d;
int pwm_output;
int toggle = HIGH;
int sensorValue = 0;        // value read from the pot
int Steer_Angle_Measure = 0;        // value output to the PWM (analog out)

float old_avg = 0.0; // Xavg(k-1)
float avg     = 0.0; // Xvag(k)

void steer_motor_control(int motor_pwm)
{
  if( (linear_mapping(avg)>= linear_mapping(AD_MAX)  && Steering_Angle >= linear_mapping(AD_MAX)) || (linear_mapping(avg) <= linear_mapping(AD_MIN) && Steering_Angle <= linear_mapping(AD_MIN)) )
  {
     digitalWrite(MOTOR3_ENA, LOW);
     digitalWrite(MOTOR3_ENB, LOW);
     //analogWrite(MOTOR3_PWM, 0);
     return;    
  }
  
  if (motor_pwm > 0) // forward
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, HIGH);
    analogWrite(MOTOR3_PWM, motor_pwm);
  }
  else if (motor_pwm < 0) // backward
  {
    digitalWrite(MOTOR3_ENA, HIGH);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, -motor_pwm);
  }
  else // stop
  {
    digitalWrite(MOTOR3_ENA, LOW);
    digitalWrite(MOTOR3_ENB, LOW);
    analogWrite(MOTOR3_PWM, 0);
  }
}

void PID_Control()
{
  error = Steering_Angle - Steer_Angle_Measure;
  error_s += error;
  error_d = error - error_old;
  error_s = (error_s >=  100) ?  100 : error_s;
  error_s = (error_s <= -100) ? -100 : error_s;

  pwm_output = Kp * error + Kd * error_d + Ki * error_s;
  pwm_output = (pwm_output >=  255) ?  255 : pwm_output;
  pwm_output = (pwm_output <= -255) ? -255 : pwm_output;

  if (fabs(error) <= 1.2)  //특정 값 이하면 제어를 멈추어서 조정이 안되도록
  {
    steer_motor_control(0);
    error_s = 0;
  }
  else          steer_motor_control(pwm_output);
  error_old = error;  
}

void steering_control()
{
  PID_Control(); 
}

double linear_mapping(double x)
{
  int i, j;
  double x1 = cal_data.X[i];
  double x2 = cal_data.X[i + 1];
  double y1 = cal_data.Y[i];
  double y2 = cal_data.Y[i + 1];
  double y;

  for (j = 0; j < No_Calibration_Point-1; j++)
  {
    if (x < cal_data.X[0])
    {
      i = 0;
    }
    else if (x >= cal_data.X[j] && x < cal_data.X[j + 1])
    {
      i = j;
    }
    else
    {
      i = No_Calibration_Point-2;
    }
    y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
  }
  return y;
}

///////////////////////////////// MsTimer callback  /////////////////////////////////////

void control_callback()
{
  static boolean output = HIGH;
  sensorValue = analogRead(Steering_Sensor);
  avg = (alpha * old_avg) + ((1.0 - alpha)*sensorValue);
  Steer_Angle_Measure = linear_mapping(avg);

////////////////////////////조종기 및 자율주행 관련//////////////////////////////////////////////

  if(mode == 1){
    target_velocity1 = m_car_speed_float.data;  //타겟 속도
    Steering_Angle = m_car_angle_int16.data+5;  //타겟 각도
    encoder1_target += target_velocity1 * vel_2_pulse;
    front_motor_PID_control();                  //PID속도 제어
  }
  else if(mode == 0){
    Steering_Angle = m_car_angle_int16.data;
  }
  
  steering_control();                         //PID각도 제어
  
  old_avg = avg;
//////////////////////////엔코더값 보내기(Front_Encoder 값만 보냄 Rear_Encoder값은 없는 경우가 많기 때문에 보내지 않음)/////////

  if(cnt%2 == 1){
    cnt=0;
    m_car_encoder_long.data = encoder1count;
    char encoder_data[7]={'#','C',m_car_encoder_long.bytedata[0],m_car_encoder_long.bytedata[1],m_car_encoder_long.bytedata[2],m_car_encoder_long.bytedata[3],'*'};
    for(int i=0; i<7; i++){
      Serial1.write(encoder_data[i]); //시리얼 1으로 데이터를 보냄
      }
    }
  cnt++;
}


///////////////////////////////// Serial_Event  /////////////////////////////////////
void serial_Event1(){  //sbus 튀는거 방지용
  if(Serial1.available())
    for(int i=0; i<9; i++){
      buf[i]= Serial1.read();
    }
    if((buf[0]=='#') &&(buf[1]=='C') && ( buf[8] == '*') )
      {
      m_car_angle_int16.bytedata[0] = buf[2];
      m_car_angle_int16.bytedata[1] = buf[3];
      
      m_car_speed_float.bytedata[0] = buf[4];      
      m_car_speed_float.bytedata[1] = buf[5];      
      m_car_speed_float.bytedata[2] = buf[6];      
      m_car_speed_float.bytedata[3] = buf[7];
      
      Serial.print(m_car_angle_int16.data);  Serial.print(" ");  Serial.print(m_car_speed_float.data);  Serial.print(" ");  Serial.print(encoder1count);  Serial.print(" ");  Serial.println(encoder2count);
    }
  }





///////////////////////////////// Setup  /////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  x8r.begin();
  Serial.begin(115200);
  Serial1.begin(38400);
  Serial.print("Serial Port Connected!");

 // Front Motor Drive Pin Setup
  pinMode(MOTOR1_PWM, OUTPUT);  // motor control PWM
  pinMode(MOTOR1_ENA, OUTPUT);  // motor control direction
  pinMode(MOTOR1_ENB, OUTPUT);  // motor control direction

  // Rear Motor Drive Pin Setup
  pinMode(MOTOR2_PWM, OUTPUT);  // motor control PWM
  pinMode(MOTOR2_ENA, OUTPUT);  // motor control direction
  pinMode(MOTOR2_ENB, OUTPUT);  // motor control direction
 
  initEncoders();          // initialize encoder
  clearEncoderCount(1); 
  clearEncoderCount(2); 

   //Steer
  pinMode(MOTOR3_PWM, OUTPUT);  // motor control PWM
  pinMode(MOTOR3_ENA, OUTPUT);  // motor control direction
  pinMode(MOTOR3_ENB, OUTPUT);  // motor control direction

  delay(500);                   //지연 시간 
  
  MsTimer2::set(50, control_callback); // 500ms period
  MsTimer2::start();

  cal_data = {
    {100.0, 180.0, 260.0, 340.0, 420.0, 500.0, 580.0, 660.0, 740.0, 820.0, 900.0},
    {-14.9, -11.3 ,-7.9, -5.4, -2.4, 1.3, 4.3, 7.9, 11.5, 15.0, 18.4} // 15, 18.7
  };
  
}

///////////////////////////////// Loop  /////////////////////////////////////

void loop() {
  // put your main code here, to run repeatedly:
  control();
  serial_Event1();
  //delay(100);
}
