#include <MsTimer2.h>
#include <NewPing.h>
#include <Servo.h>
#include <Wire.h>
#include "sbus.h"

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////// i2c_Control /////////////////////////
#define SLAVE_ADDRESS 0x05

#define NO_OF_RECEIVED_BUFFER        7
#define NO_OF_SEND_EncoderPos_BUFFER 7

unsigned char EncoderPos_data[NO_OF_SEND_EncoderPos_BUFFER] = {'#', 'D', 0, 0, 0, 0, '*'}; // 시작 바이트 '#' - 끝 바이트 '*'

////////////////////////// Sbus /////////////////////////
#define Sbus_Speed_max  1800
#define Sbus_Speed_zero  974
#define Sbus_Speed_min   210

#define Sbus_Steer_max  1800
#define Sbus_Steer_zero 1002
#define Sbus_Steer_min   200

#define Sbus_offset       30

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial1);
/* SBUS object, writing SBUS */
bfs::SbusTx sbus_tx(&Serial1);
/* SBUS data */
bfs::SbusData data;

struct MotorData
{
  int motor_speed;
  int str_position;
};
MotorData MD;

union
{
  short data;
  byte bytedata[2];
} m_car_angle_int16;

union
{
  int data;
  byte bytedata[2];
} m_car_speed_int16, EncoderPos;

union
{
  float data;
  byte bytedata[4];
} m_car_sonar_front_left, m_car_sonar_front_right;

////////////////////// Control_Mode //////////////////////
#define FULL_RC_CONTROL              0
#define FULL_AUTONOMOUS              1
#define SEMI_AUTONOMOUS              2

int control_mode   = 0;
int DEBUG          = 1;
int SBUS_Max_Speed = 0;

bool sbus_connected = false;
unsigned long last_sbus_time = 0;
const unsigned long sbus_timeout = 1000; // 1초 타임아웃

///////////////////////// MOTOR /////////////////////////
#define SPEED_SERVO_PIN  8

#define NEURAL_SPEED 90
#define SPEED_OFFSET 0

#define MAX_SPEED 150
#define MIN_SPEED 30

#define NEURAL_MOTOR_SPEED    0

Servo servo_speed;

int motor_speed        = 0;

void map_Motor_Speed()
{
  if (MD.motor_speed > 90)
  {
    motor_speed = map(MD.motor_speed, NEURAL_SPEED, MAX_SPEED, NEURAL_MOTOR_SPEED, SBUS_Max_Speed);
  }

  else if (MD.motor_speed < 90)
  {
    motor_speed = map(MD.motor_speed, MIN_SPEED, NEURAL_SPEED, -SBUS_Max_Speed, NEURAL_MOTOR_SPEED);
  }

  else
  {
    motor_speed = 0;
  }
}

////////////////////////// STEER /////////////////////////
#define STEER_SERVO_PIN  4

#define NEURAL_ANGLE 90
#define ANGLE_OFFSET  8

#define LEFT_ANGLE   55
#define RIGHT_ANGLE 125

#define NEURAL_STEER_ANGLE   0
#define LEFT_STEER_ANGLE   -35
#define RIGHT_STEER_ANGLE   35

Servo servo_steer;

int steering_angle = 0;

void map_Steer_Angle()
{
  if (MD.str_position > 90)
  {
    steering_angle = map(MD.str_position, NEURAL_ANGLE, RIGHT_ANGLE, NEURAL_STEER_ANGLE, RIGHT_STEER_ANGLE);
  }

  else if (MD.str_position < 90)
  {
    steering_angle = map(MD.str_position, LEFT_ANGLE, NEURAL_ANGLE, LEFT_STEER_ANGLE, NEURAL_STEER_ANGLE);
  }

  else
  {
    steering_angle = 0;
  }
}

/////////////////////////// Sonar //////////////////
#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 2000 // Maximum distance (in cm) to ping.

NewPing sonar[SONAR_NUM] =
{ // Sensor object array.
  NewPing(30, 31, MAX_DISTANCE), // trig echo
  NewPing(32, 33, MAX_DISTANCE), // trig echo
};

unsigned long previousMillis = 0;
const long interval          = 200;

/////////////////////////// Pulse  //////////////////////
// 추후에 계산 필요
#define pulse 232.0
#define pulse_m 1160.0
double  pulse_distance;

////////////////////////// ENCODER /////////////////////
#define Encoder_PinA 2
#define Encoder_PinB 3

volatile long Encoder_Pos = 0;

////////////////////////// Ms2Timer //////////////////
void Ms2Timer_setup(void)
{
  MsTimer2::set(10, MsTimerISR);
  MsTimer2::start();
}

////////////////////////// Setup //////////////////
void setup()
{
#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  servo_speed.attach(SPEED_SERVO_PIN);
  servo_steer.attach(STEER_SERVO_PIN);

  servo_speed.write(NEURAL_SPEED);
  servo_steer.write(NEURAL_ANGLE);

  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(requestData);

  while (!Serial) {}
  sbus_rx.Begin();
  sbus_tx.Begin();

  sbus_connected = false;
  last_sbus_time = millis();
  
  Ms2Timer_setup();

  Serial.begin(115200);
}

/////////////////////////////////////// Sonar ////////////////////////
void read_ultrasonic_sensor(void)
{
  m_car_sonar_front_left.data  = sonar[0].ping_cm() * 10.0;  // 전방
  m_car_sonar_front_right.data = sonar[1].ping_cm() * 10.0;  // 왼쪽

  if (  m_car_sonar_front_left.data == 0)
  {
    m_car_sonar_front_left.data = MAX_DISTANCE;
  }

  if ( m_car_sonar_front_right.data == 0)
  {
    m_car_sonar_front_right.data = MAX_DISTANCE;
  }
}

void ultrasonic_sensor_serial_print(void)
{
  read_ultrasonic_sensor();

  Serial.print("L_sonar: ");
  Serial.print(m_car_sonar_front_left.data);
  Serial.print(" ");

  Serial.print("R_sonar: ");
  Serial.print(m_car_sonar_front_right.data);
  Serial.println(" ");
}

/////////////////////////////////////// ENCODER ////////////////////////
void Encoder_B()
{
  delayMicroseconds(2);

  if (digitalRead(Encoder_PinB) == LOW)
  {
    Encoder_Pos++;
  }
  else
  {
    Encoder_Pos--;
  }
}

void Encoder_Setup(void)
{
  pinMode(Encoder_PinA, INPUT_PULLUP);        // quadrature encoder input A
  pinMode(Encoder_PinB, INPUT_PULLUP);        // quadrature encoder input B
  attachInterrupt(0, Encoder_B, FALLING);    // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;         // To prevent Motor Noise
}

void reset_encoder(void)
{
  Encoder_Pos = 0;
}

void Encoder_Print(void)
{
  Serial.print("Encoder_Pos: ");
  Serial.println(Encoder_Pos);
}

void pulse_conversion(void)
{
  Encoder_B();
  pulse_distance = Encoder_Pos / pulse_m;
}

////////////////////////////////////// i2c_Control /////////////////////
void receiveData(int byteCount)
{
  if (Wire.available() >= 9)
  {
    byte receivedData[9];
    for (int i = 0; i < 9; i++)
    {
      receivedData[i] = Wire.read();
    }

    if (receivedData[0] == '#' && receivedData[1] == 'C' && receivedData[8] == '*')
    {
      m_car_angle_int16.bytedata[0] = receivedData[2];
      m_car_angle_int16.bytedata[1] = receivedData[3];
      //short angle = m_car_angle_int16.data;

      m_car_speed_int16.bytedata[0] = receivedData[4];
      m_car_speed_int16.bytedata[1] = receivedData[5];
      //int speed = m_car_speed_int16.data;

      //delay(1000);
    }
    else
    {
      Serial.println("Invalid protocol");
    }
  }
}

void requestData()
{
  EncoderPos.data = Encoder_Pos; 
  EncoderPos_data[0] = '#';
  EncoderPos_data[1] = 'D';
  EncoderPos_data[2] = EncoderPos.bytedata[0]; 
  EncoderPos_data[3] = EncoderPos.bytedata[1];
  EncoderPos_data[4] = 0;
  EncoderPos_data[5] = 0;
  EncoderPos_data[6] = '*';

  Wire.write(EncoderPos_data, sizeof(EncoderPos_data));
}

/////////////////////////////// Sbus /////////////////
void sbus_control()
{
  if (sbus_rx.Read())
  {
    data = sbus_rx.data();

    sbus_connected = true;
    last_sbus_time = millis();

    /*
        for (int8_t i = 0; i < 20; i++)
        {
          Serial.print(data.ch[i]);
          Serial.print("\t");
        }
        Serial.println("");
    */

    sbus_tx.data(data);
    sbus_tx.Write();

    if (data.ch[4] >= 1600)
    {
      SBUS_Max_Speed = 255;
    }
    else if (data.ch[4] <= 300)
    {
      SBUS_Max_Speed = 100;
    }
    else
    {
      SBUS_Max_Speed = 150;
    }

    if (data.ch[2] >= Sbus_Speed_zero + Sbus_offset)
    {
      MD.motor_speed = map(data.ch[2], Sbus_Speed_zero, Sbus_Speed_max, NEURAL_SPEED, MAX_SPEED);
    }
    else if (data.ch[2] < Sbus_Speed_zero - Sbus_offset)
    {
      MD.motor_speed = map(data.ch[2], Sbus_Speed_min, Sbus_Speed_zero, MIN_SPEED, NEURAL_SPEED);
    }
    else
    {
      MD.motor_speed = 90;
    }

    if (data.ch[0] >= Sbus_Steer_zero + Sbus_offset)
    {
      MD.str_position = map(data.ch[0], Sbus_Steer_zero, Sbus_Steer_max, NEURAL_ANGLE, RIGHT_ANGLE);
    }
    else if (data.ch[0] <= Sbus_Steer_zero - Sbus_offset)
    {
      MD.str_position = map(data.ch[0], Sbus_Steer_min, Sbus_Steer_zero, LEFT_ANGLE, NEURAL_ANGLE);
    }
    else
    {
      MD.str_position = 90;
    }

    if (data.ch[6] >= 1600)
    {
      control_mode = SEMI_AUTONOMOUS;
    }
    else if (data.ch[6] <= 300)
    {
      control_mode = FULL_RC_CONTROL;
    }
    else
    {
      control_mode = FULL_AUTONOMOUS;
    }
  }
}
void joy_control()
{
  // SBUS 연결 해제 여부 확인
  if (millis() - last_sbus_time > sbus_timeout)
  {
    sbus_connected = false;
  }

  // SBUS가 연결되지 않은 경우 FULL_AUTONOMOUS 모드로 강제 전환
  if (!sbus_connected)
  {
    control_mode = FULL_AUTONOMOUS;
  }

  switch (control_mode)
  {
    case FULL_RC_CONTROL :
      servo_speed.write(motor_speed);
      servo_steer.write(steering_angle);

      if (DEBUG)
      {
        Serial.println("<Full RC>");
        Serial.print("speed : ");
        Serial.print(motor_speed);
        Serial.print("\tsteer : ");
        Serial.println(steering_angle);
        Serial.println(" ");
      }
      break;

    case FULL_AUTONOMOUS:
    default:  // control_mode가 유효하지 않을 경우 FULL_AUTONOMOUS 사용을 보장하기 위해 default 케이스 추가
      servo_speed.write(m_car_speed_int16.data);
      servo_steer.write(m_car_angle_int16.data);

      if (DEBUG)
      {
        Serial.println("<Full Auto>");
        Serial.print("speed : ");
        Serial.print(m_car_speed_int16.data);
        Serial.print("\tsteer : ");
        Serial.println(m_car_angle_int16.data);
        Serial.println(" ");
      }
      break;

    case SEMI_AUTONOMOUS :
      servo_speed.write(motor_speed);
      servo_steer.write(m_car_angle_int16.data);

      if (DEBUG)
      {
        Serial.println("<Semi Auto>");
        Serial.print("speed : ");
        Serial.print(motor_speed);
        Serial.print("\tsteer : ");
        Serial.println(m_car_angle_int16.data);
        Serial.println(" ");
      }
      break;
  }
}

void MsTimerISR()
{
  Encoder_Setup();
  map_Steer_Angle();
  map_Motor_Speed();
}

void loop ()
{
  unsigned long currentMillis = millis();

  sbus_control();
  joy_control();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;

    read_ultrasonic_sensor();
    ultrasonic_sensor_serial_print();
  }
}
