#include <Servo.h>
#include <MsTimer2.h>

#define LINE_PRINT    1  //serial 출력을 위한 것
#define encodPinA1    2
#define encodPinB1    3
#define MOTOR_DIR     4
#define MOTOR_PWM     5
#define A0pin        A0
#define SIpin        13
#define CLKpin       12
#define RC_SERVO_PIN  8

#define LEFT_STEER_ANGLE   -35    //왼쪽   서보 최대 각도
#define RIGHT_STEER_ANGLE   25    //오른쪽 서보 최대 각도
#define NEURAL_ANGLE        90    //기본 중립
#define NEURAL_ANGLE_offset -8    //중립을 맞추기 위한 offset

#define threshold_valus  60
#define NPIXELS         128
#define OFFSET            0

float Kp  = 0.6;
float Ki  = 0;
float Kd  = 1.2;

byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor    

int Steering_Angle = 0;
int steer_data = 0;

int Line_Center       = NPIXELS/2;
int Line_L_Center     = 10;
int Line_R_Center     = NPIXELS-10;

int Line_L_Center_old = 10;
int Line_R_Center_old = NPIXELS-10;


float error     =0;
float error_s   =0;
float error_d   =0;
float error_old =0;

//double Line_L_Center;
//double Line_R_Center;
  

volatile long encoderPos = 0;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

Servo Steeringservo;

int steering_control()   
{
  // 서보 angle 값 제한한
  if(Steering_Angle <= LEFT_STEER_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE;
  if(Steering_Angle >= RIGHT_STEER_ANGLE) Steering_Angle = RIGHT_STEER_ANGLE;
  
  Steeringservo.write(Steering_Angle + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void encoderB()  
{
  delayMicroseconds(2);
  if(digitalRead(encodPinB1)==LOW)    encoderPos++;           
  else                                encoderPos--; 
}

void line_adaptation(void)
{
  int i;
  for(i=0; i<NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i]) MAX_LineSensor_Data[i]=LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i]) MIN_LineSensor_Data[i]=LineSensor_Data[i];
  }
}
  
void read_line_sensor(void)
{
  int i;
  delayMicroseconds (1);
  delay(10);

  digitalWrite (CLKpin,LOW);
  digitalWrite (SIpin,HIGH);
  digitalWrite (CLKpin,HIGH);
  digitalWrite (SIpin,LOW);

  delayMicroseconds (1);

  for(i=0; i<NPIXELS; i++)
  {
    Pixel[i]=analogRead (A0pin);
    digitalWrite (CLKpin,LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin,HIGH);
  }
  for(i=0; i<NPIXELS; i++)
  {
    LineSensor_Data_Adaption[i] = map(Pixel[i],MIN_LineSensor_Data[i],MAX_LineSensor_Data[i],0,256);
  }

  if(LINE_PRINT != 1)  return;
  for(i=0; i<NPIXELS; i++)
  {
   
    Serial.print(LineSensor_Data_Adaption[i]);
    //Serial.print((byte)Pixel[i]);
    Serial.print(" ");
    Serial.print(Line_L_Center);  // 노란색
    Serial.print(" ");
    Serial.print(Line_R_Center);  // 파란색
    Serial.print(" ");     
    Serial.print(Line_Center);    // 붉은 색
    Serial.println(" ");
  }
}

void threshold(void){
  int i;
  for(i=0; i<NPIXELS; i++)
  {
    if(LineSensor_Data_Adaption[i]>=threshold_valus)
    {
      LineSensor_Data_Adaption[i]=255;
     }
    else
    {
      LineSensor_Data_Adaption[i]=0;
     }
    }
}

void find_line_center(void)
{
  int i;
  long sum=0;
  long x_sum=0;
  int distance_L = 0; int distance_R = 0;
  
  for(i=0; i<NPIXELS; i++)
  {
    sum += LineSensor_Data_Adaption[i];
    x_sum += (LineSensor_Data_Adaption[i])*i;
  }
  Line_Center = (x_sum/sum);

  if(LineSensor_Data_Adaption[Line_Center] != 255)  // Line이 2개 검출 될때 
  {
      for (i = 0,sum = 0,x_sum = 0; i < Line_Center; i++)
      {
         sum  += LineSensor_Data_Adaption[i];
         x_sum += LineSensor_Data_Adaption[i]*i;
      }

      if(sum !=  0) Line_L_Center = (x_sum/sum);

      for (i = Line_Center,sum = 0,x_sum = 0; i < NPIXELS; i++)
      {
        sum += LineSensor_Data_Adaption[i];
        x_sum += LineSensor_Data_Adaption[i]*i;
      }
   
     if(sum !=  0) Line_R_Center = (x_sum/sum);

     Line_Center = (Line_R_Center + Line_L_Center)/2;
  }
  else
  {
     distance_L = abs(Line_Center - Line_L_Center_old);
     distance_R = abs(Line_Center - Line_R_Center_old); 
     if(distance_L < distance_R)  
     {
        Line_L_Center = Line_Center;
        Line_R_Center = Line_L_Center+60;
     }
          
     if(distance_L > distance_R)  
     {
        Line_R_Center = Line_Center;
        Line_L_Center = Line_R_Center - 60;
     }
          
     Line_Center = (Line_R_Center + Line_L_Center)/2;
  }
  
  Line_L_Center_old = Line_L_Center; 
  Line_R_Center_old = Line_R_Center;
          
}

void PID_line_control()
{
  
  error   = (Line_Center - NPIXELS/2 + OFFSET)*1 ;
  error_d = error - error_old;
  error_s = (error_s >= 5) ?   5 : error_s;
  error_s = (error_s <= -5) ? -5 : error_s;

  Steering_Angle = Kp*error + Kd*error_d + Ki*error_s;

  if(fabs(error) <= 1)
  {
    error_s = 0;
  }
  steering_control();
  error_old = error;
  }

void motor_control(int speed)
{
  if(speed >= 0){
    digitalWrite(MOTOR_DIR,HIGH);
    analogWrite(MOTOR_PWM,speed);
    }
  else{
    digitalWrite(MOTOR_DIR,LOW);
    analogWrite(MOTOR_PWM,-speed);
    }
  }

void CallBack()
{
  line_adaptation();
  read_line_sensor();
  threshold();
  find_line_center();
  PID_line_control();
  motor_control(130);  // 속도
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoder, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise  
}

void setup() {
  // put your setup code here, to run once:
  for (int a=0; a<NPIXELS; a++){
    LineSenSor_Data[a] = 0;
    LineSenSor_Data_Adaption[a] = 0;
    MAX_LineSensor_Data[a] = 1023;
    MIN_LineSensor_Data[a] = 0;
   }
   
  pinMode(SIpin,OUTPUT);
  pinMode(CLKpin,OUTPUT);
  digitalWrite(CLKpin,LOW);
  digitalWrite(SIpin,LOW);

  #if FASTADC
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);
  #endif

  pinMode(MOTOR_DIR,OUTPUT);
  pinMode(MOTOR_PWM,OUTPUT);

  interrupt_setup();

  cal_data = {
    {-24.1,-21.95,-19.55,-16.95,-13.95,-10.8,-8.4,-4.9,3.35,5.5,9.1,12.9,16.1,19.8,23.1,26.3,29.3},
    {130,125,120,115,110,105,100,95,90,85,80,75,70,65,60,55,50}
  };

  
  MsTimer2::set(10, CallBack);
  MsTimer2::start();
  
  Serial.begin(115200);
  
  delay(500);
}

void loop() 
{
  // put your main code here, to run repeatedly:

}
