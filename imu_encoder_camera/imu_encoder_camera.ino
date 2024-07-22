//////////////////////// camera & line_control (TSL1401) ///////////////////////
#define A0pin   A0
#define SIpin   22
#define CLKpin  23
#define NPIXELS 128

double kp_vision;
double ki_vision;
double kd_vision;
double error      = 0.0;
double error_old  = 0.0;
double Target     = NPIXELS / 2;

byte Pixel[NPIXELS];
byte LineSensor_threshold_Data[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];

/////////////////////////// IMU //////////////////////////
#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

#define Waypoint_Angle_Tor 2

LSM303 compass;

float heading_angle        = 0.0;
float target_heading_angle = 0.0;
float heading_angle_error  = 0.0;

///////////////////// L298N ///////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

//////////////////////////// etc ////////////////////////////
#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

const byte outPin     = 13;
const byte encoder1_A = 2;
const byte encoder1_B = 3;
const byte encoder2_A = 18;
const byte encoder2_B = 19;
volatile byte state   = 0;
long cnt1             = 0;
long cnt2             = 0;

////////////////////////////////// encoder ////////////////////////////////
void intfunc1()
{
  if (digitalRead(encoder1_B) == HIGH)
  {
    cnt1--;
  }
  else
  {
    cnt1++;
  }
}

void intfunc2()
{
  if (digitalRead(encoder2_B) == LOW)
  {
    cnt2--;
  }
  else
  {
    cnt2++;
  }
}

///////////////////////// setup //////////////////////////
void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(outPin, OUTPUT);
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING);

  Serial.begin(115200);

  Wire.begin();
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>)
  {
    -32767, -32767, -32767
  };

  compass.m_max = (LSM303::vector<int16_t>)
  {
    +32767, +32767, +32767
  };

  for (int i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023;
    MIN_LineSensor_Data[i] = 0;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(A0pin, INPUT);

  digitalWrite(SIpin, LOW);
  digitalWrite(CLKpin, LOW);

#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif
}

/////////////////////////// motor control /////////////////////////////
void motor_R_control(int direction_r, int motor_speed_r)
{
  if (direction_r == HIGH)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motor_speed_r);

  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed_r);

  }
}

void motor_L_control(int direction_l, int motor_speed_l)
{
  if (direction_l == HIGH)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motor_speed_l);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, motor_speed_l);
  }
}
void motor_control_R(int pwm_r)
{
  if (pwm_r >= 0 )
  {
    motor_R_control(1, pwm_r);
  }
  else
  {
    motor_R_control(0, -pwm_r);
  }
}

void motor_control_L(int pwm_l)
{

  if (pwm_l >= 0 )
  {
    motor_L_control(1, pwm_l);
  }
  else
  {
    motor_L_control(0, -pwm_l);
  }
}

//////////////////////////// IMU /////////////////////////////
void read_imu_sensor(void)
{
  compass.read();
  float heading1 = compass.heading();
  delay(50);
  compass.read();
  float heading2 = compass.heading();
  heading_angle = (heading1 + heading2) / 2;
  heading_angle = 360 - heading_angle;
  heading_angle_error = target_heading_angle - heading_angle;

  if (heading_angle_error > 180)
  {
    heading_angle_error = heading_angle_error - 360;
  }
  else if (heading_angle_error < -180)
  {
    heading_angle_error = heading_angle_error + 360;
  }
  else
  {
    // 다른 작업 수행
  }

  Serial.print("Heading Angle: ");
  Serial.println(heading_angle);
}

void imu_rotation(int base_speed)
{
  bool flag = 1; // bool 타입은 true(1) 또는 false(0)
  while (flag)
  {
    int temp_data = 0;
    temp_data = (base_speed) * 2.5;
    if (temp_data >= 255) temp_data = 255;

    read_imu_sensor();

    if (heading_angle_error > THRESHOLD_ANGLE1) // 반시계방향으로 회전
    {
      motor_control_L(-temp_data);
      motor_control_R(temp_data);
    }
    else if ((heading_angle_error >= THRESHOLD_ANGLE2) && (heading_angle_error <= THRESHOLD_ANGLE1)) //
    {
      motor_control_L(-temp_data);
      motor_control_R(temp_data);
    }
    else if ((heading_angle_error > -THRESHOLD_ANGLE2) && (heading_angle_error < THRESHOLD_ANGLE2)) // 정지
    {
      motor_control_L(0);
      motor_control_R(0);
      flag = false; // 루프를 빠져나오도록 설정
    }
    else if ((heading_angle_error >= -THRESHOLD_ANGLE1) && (heading_angle_error <= -THRESHOLD_ANGLE2)) //
    {
      motor_control_L(temp_data);
      motor_control_R(-temp_data);
    }
    else // heading_angle_error <-THRESHOLD_ANGLE // 시계방향으로 회전
    {
      motor_control_L(temp_data);
      motor_control_R(-temp_data);
    }
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
  }
}


////////////////////////// TSL 1401 /////////////////////////
void threshold_line_image(int threshold_value)
{
  for (int i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_value)
    {
      LineSensor_threshold_Data[i] = 255;
    }
    else
    {
      LineSensor_threshold_Data[i] = 0;
    }
  }
}

void read_line_camera(void)
{
  int i;
  delay(1);

  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, HIGH);
  digitalWrite(CLKpin, HIGH);
  digitalWrite(SIpin, LOW);
  delayMicroseconds(1);

  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead(A0pin) / 4;//8-bit is enough       *0.25
    digitalWrite(CLKpin, LOW);
    delayMicroseconds(1);
    digitalWrite(CLKpin, HIGH);
  }
  digitalWrite(CLKpin, LOW);
}

double line_centroid(void)
{
  double centroid = 0.0;
  double mass_sum = 0;

  for (int i = 0; i < NPIXELS; i++)
  {
    mass_sum += LineSensor_threshold_Data[i];
    centroid += LineSensor_threshold_Data[i] * i;
  }

  centroid = centroid / mass_sum;

  return centroid;
}

double PID_control(double line_center)
{
  kp_vision = 4.0;   // 1.0
  ki_vision = 0.0;
  kd_vision = 3.0;     // 3.0 이상

  //kp_vision = 1.0기준
  //kd_vision = 3.0이상 1.8 2.0 2.5

  //0.5         3.0
  //0.5         2.5 (부드러움)
  //1.0         1.8,2.0 (부드러움)
  //3.0~5.0     2.45
  //2.0         2.5,2.8(부드러)

  int pwm_value = 0;
  double error_d = 0;
  error = Target - line_center;
  error_d = error - error_old;
  pwm_value = int(kp_vision * error + kd_vision * error_old);
  if (pwm_value >=  255) pwm_value = 255;
  if (pwm_value <= -255) pwm_value = -255;

  error_old = error;

  return pwm_value;
}

void vision_line_control(int base_speed, double l_c)
{
  int pwm_control_value = PID_control(l_c);
  int temp_data = 0;
  temp_data = (base_speed - pwm_control_value) * 2.5;
  if (temp_data >= 255) temp_data = 255;
  //if (temp_data <= -50) temp_data = -50;
  motor_control_L(temp_data);  // 부호는 반대로

  temp_data = (base_speed + pwm_control_value) * 2.5;
  if (temp_data >= 255) temp_data = 255;
  //if (temp_data <= -50) temp_data = -50;
  motor_control_R(temp_data);
}

void VLC_move(void)  // 비전 라인 컨트롤
{
  int cx_int;
  double cx = 0;

  cx_int = (int)cx;
  read_line_camera();
  threshold_line_image(30);  // 값 조정
  cx = line_centroid();
  vision_line_control(130, cx);

  /*
    base_speed < 105, kp = 1.0, kd = 3.0
    base_speed < 110, kp = 0.1, kd = 3.0
  */

  ////// 디버깅하는데에만 필요하지 나중에는 cx만 남기고 지워야 됨 ///////
  for (int i = 0; i < NPIXELS; i++)
  {
    //   Serial.print(LineSensor_threshold_Data[i] + 1);
    // Serial.print(" ");
    Serial.println(cx_int);
  }


  //Serial.print(cx_int);
  Serial.print(" ");
  Serial.println(" ");
  // Serial.println(cx);
}

int state_vlc  = 0; // Initial state
int waypointReached = 0; // Variable to track the current waypoint

void loop()
{
  switch (state_vlc)
  {
    case 0:
      if (cnt1 < 3350)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 1;
      }
      break;

    case 1: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 356;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 2;
        cnt1 = 0;
      }
      break;

    case 2:
      if (cnt1 < 3800)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 3;
      }
      break;

    case 3: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 80;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 4;
        cnt1 = 0;
      }
      break;

    case 4:
      if (cnt1 < 3900)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 5;
      }
      break;

    case 5: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 350;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 6;
        cnt1 = 0;
      }
      break;

    case 6:
      if (cnt1 < 3874)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 7;
      }
      break;

    case 7: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 270;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 8;
        cnt1 = 0;
      }
      break;

    case 8:
      if (cnt1 < 3690)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 9;
      }
      break;

    case 9: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 189;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 10;
        cnt1 = 0;
      }
      break;

    case 10:
      if (cnt1 < 3670)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 11;
      }
      break;

    case 11: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 80;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 12;
        cnt1 = 0;
      }
      break;

    case 12:
      if (cnt1 < 3782)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 13;
      }
      break;

    case 13: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 189;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 14;
        cnt1 = 0;
      }
      break;

    case 14:
      if (cnt1 < 3830)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 15;
      }
      break;

    case 15: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 283;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 16;
        cnt1 = 0;
      }
      break;

    case 16:
      if (cnt1 < 4029)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 17;
      }
      break;

    case 17: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 357;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 18;
        cnt1 = 0;
      }
      break;

    case 18:
      if (cnt1 < 8200)
      {
        VLC_move();
      }
      else
      {
        motor_control_L(0);
        motor_control_R(0);
        state_vlc  = 19;
      }
      break;

    case 19: // Rotate to reach the desired IMU angle
      read_imu_sensor();
      target_heading_angle = 80;

      if (abs(heading_angle - target_heading_angle) > Waypoint_Angle_Tor)
      {
        imu_rotation(255);
        state_vlc  = 20;
        cnt1 = 0;
      }
      break;
    case 20:
      if (cnt1 < 7200)
      {
        VLC_move();
      }

  }
}
