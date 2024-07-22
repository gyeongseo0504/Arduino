#include <MsTimer2.h>

/////////////// WayPoint Control ///////////////////
#define VLC 0
#define IHC 1

#define X_GRID_LENGTH 100
#define Y_GRID_LENGTH 120
#define MAX_Waypint_NO 50

#define Waypoint_X_Tor   0.1
#define Waypoint_Y_Tor   0.1
#define Waypoint_Angle_Tor 2

int no_waypoints = 0;
int wp_go_id     = 0;
int count;

struct Waypoint
{
  double pos_x;
  double pos_y;
  double angle;
  int control_type;
};

struct Waypoint waypoint[MAX_Waypint_NO];

//////////////////// BlueTooth /////////////////////
#define BTSerial Serial2
#define RxD 17
#define TxD 16
#define BT_BAUDRATE 9600


//////////////////////// camera & line_control (TSL1401) //////////////////////////
#define A0pin A0
#define SIpin 22    //Start Integration ~ orange
#define CLKpin 23
#define NPIXELS 128 // No. of pixels in array

double kp_vision = 0.5;
double ki_vision = 0.3;
double kd_vision = 5.0;
double error     = 0.0;
double error_old = 0.0;
double Target    = NPIXELS / 2;

byte Pixel[NPIXELS]; // Field for measured values <0~255>
byte LineSensor_threshold_Data[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];
int MAX_LineSensor_Data[NPIXELS];
int MIN_LineSensor_Data[NPIXELS];
int flag_line_adapation;

#define FASTADC 1
//defines for setting and clearing register bits
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

/////////////////////////// IMU //////////////////////////
#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

LSM303 compass;

float heading_angle        = 0.0;
float init_heading_        = 17.0;   // 초기 방향
float target_heading_angle = 90;
float heading_angle_error  = 0.0;  // error 값

///////////////////// L298N ///////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

//////////////////////// odom & yaw ///////////////////////

#define RAD2DEG(x)   (x*180.0/3.14159)
#define DEG2RAD(x)   (x*3.14159/180.0)
#define wheel_track 0.068 //양쪽 바뀌       // m 단위로 구할 것 0.1 -> 10cm

struct Pose2D
{
  double x;
  double y;
  double theta; //theta = radian
} my_odom;

const byte outPin = 13; // Output pin: digital pin 13(D13)
//const byte interruptPin1 = 20; // Interrupt pin: D2
//const byte interruptPin2 = 21; // Interrupt pin: D2
const byte encoder1_A = 2; // Interrupt pin: D2
const byte encoder1_B = 3; // Interrupt pin: D2
const byte encoder2_A = 18; // Interrupt pin: D2
const byte encoder2_B = 19; // Interrupt pin: D2
const byte resetPin = 5;
volatile byte state = 0;
//unsigned long cnt1 = 0; // 추가
//unsigned long cnt2 = 0; // 추가
long cnt1 = 0; // 추가
long cnt2 = 0; // 추가

long cnt1_old = 0; // 추가
long cnt2_old = 0; // 추가

double pulst_to_distance_left = 0.2 / 974.6;
double pulst_to_distance_right = 0.2 / 974.5;

const double odom_left = 0;
const double odom_right = 0;

double yaw = 0.0;
double yaw_degree = 0.0;
double yaw1 = 0.0; // 추가

double heading(double x, double y)
{
  double head = atan2(y, x); // Slope Y, Slope X  x = wheel_track
  return head;
}

void MsTimer2_ISR()
{
  double odom_left_delta = 0.0;
  double odom_right_delta = 0.0;
  double odom_center_delta = 0.0;
  long delta_encoder_left = 0;
  long delta_encoder_right = 0;
  double theta_delta = 0.0;
  double theta_delta_degree = 0.0;

  delta_encoder_left = cnt1 - cnt1_old;
  delta_encoder_right = cnt2 - cnt2_old;

  // Serial.print("theta_delta = "); Serial.print(theta_delta); Serial.print("    ");

  theta_delta_degree = DEG2RAD(theta_delta);
  //  Serial.print("theta_delta_degree = "); Serial.print(theta_delta_degree); Serial.print("    ");

  odom_left_delta = delta_encoder_left * pulst_to_distance_left;
  odom_right_delta = delta_encoder_right * pulst_to_distance_right;
  odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5; // 안전성 나누기 지원 안하는 것도 있고 속도도 조금 더 빠르기 때문

  //  Serial.print("delta_encoder_left = ");  Serial.print( delta_encoder_left);  Serial.print("    ");
  // Serial.print("delta_encoder_right = "); Serial.print( delta_encoder_right); Serial.println("    ");

  // Serial.print("odom_left_delta = ");  Serial.print(odom_left_delta);  Serial.print("    ");
  //  Serial.print("odom_right_delta = "); Serial.print(odom_right_delta); Serial.println("    ");

  theta_delta = heading(wheel_track, (odom_right_delta - odom_left_delta));
  // Serial.print("delta theta radian: ");  Serial.print(theta_delta); Serial.println("    ");
  yaw += theta_delta;
  theta_delta_degree = RAD2DEG(theta_delta);
  // Serial.print("yaw radian: ");  Serial.print(yaw); Serial.println("    ");
  // Serial.print("delta theta degree: ");  Serial.print(theta_delta_degree); Serial.println("    ");
  yaw_degree += theta_delta_degree;
  // Serial.print("yaw degree: ");  Serial.print(yaw_degree); Serial.println("    ");

  my_odom.x += odom_center_delta * sin(theta_delta);
  my_odom.y += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta;

  // Serial.print("odom : ");
  // Serial.print(my_odom.x); Serial.print("    ");
  // Serial.print(my_odom.y); Serial.print("    ");
  //Serial.println(my_odom.theta); Serial.println("    ");

  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

void update_odometry()
{
  // 엔코더 값에서 이동 거리를 계산합니다.
  double delta_encoder_left = cnt1 - cnt1_old;
  double delta_encoder_right = cnt2 - cnt2_old;
  double odom_left_delta = delta_encoder_left * pulst_to_distance_left;
  double odom_right_delta = delta_encoder_right * pulst_to_distance_right;
  double odom_center_delta = (odom_left_delta + odom_right_delta) * 0.5;

  // IMU에서 측정한 방향을 라디안으로 변환합니다.
  double theta_delta = DEG2RAD(yaw1);

  // 로봇의 위치와 방향을 업데이트합니다.
  my_odom.x += odom_center_delta * sin(theta_delta);
  my_odom.y += odom_center_delta * cos(theta_delta);
  my_odom.theta += theta_delta;

  // 업데이트된 엔코더 값 저장
  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(outPin, OUTPUT); // Output mode
  //  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  //  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  //  attachInterrupt(digitalPinToInterrupt(interruptPin1), intfunc1, RISING); // Enable interrupt
  //  attachInterrupt(digitalPinToInterrupt(interruptPin2), intfunc2, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt

  my_odom.x = 0;   my_odom.y = 0;   my_odom.theta = 0;

  Serial.begin(115200);
  BTSerial.begin(BT_BAUDRATE);

  MsTimer2::set(100, MsTimer2_ISR); // 0.1초마다 작동
  MsTimer2::start();

  Wire.begin();  // IMU initiallize
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

  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    LineSensor_Data[i] = 0;
    LineSensor_Data_Adaption[i] = 0;
    MAX_LineSensor_Data[i] = 1023; //0;
    MIN_LineSensor_Data[i] = 0; //1023;
  }
  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  pinMode(A0pin, INPUT);

  digitalWrite(SIpin, LOW);   // IDLE state
  digitalWrite(CLKpin, LOW);  // IDLE state

#if FASTADC
  // set prescale to 16
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  flag_line_adapation = 0;

  Serial.begin(115200);
  Serial.println("TSL1401");
}

/////////////////////////// motor control /////////////////////////////
void motor_R_control(int direction_r, int motor_speed_r) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if (direction_r == LOW)
  {
    digitalWrite(IN1, LOW); //모터의 방향 제어
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motor_speed_r); //모터의 속도 제어

  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed_r);

  }
}

void motor_L_control(int direction_l, int motor_speed_l) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if (direction_l == HIGH)
  {
    digitalWrite(IN3, HIGH); //모터의 방향 제어
    digitalWrite(IN4, LOW);
    analogWrite(ENB, motor_speed_l); //모터의 속도 제어
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
    //Serial.println("Right forward");
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
    //Serial.println("Left forward");
    motor_L_control(1, pwm_l);
  }
  else
  {
    motor_L_control(0, -pwm_l);
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
  int pwm_value = 0;
  double error_d = 0;
  error = Target - line_center;
  error_d = error - error_old;
  pwm_value = int(kp_vision * error + kd_vision * error_old);
  if (pwm_value >=  200) pwm_value = 200;
  if (pwm_value <= -200) pwm_value = -200;

  error_old = error;

  return pwm_value;
}

void vision_line_control(int base_speed, double l_c)
{
  int pwm_control_value = PID_control(l_c);
  motor_control_L(base_speed - pwm_control_value);  // 부호는 반대로
  motor_control_R(base_speed * 1.3 + pwm_control_value);
}

////////////////////////// IMU /////////////////////////////
void read_imu_sensor(void)
{
  compass.read();
  float heading1 = compass.heading();
  compass.read();
  float heading2 = compass.heading();
  heading_angle = (heading1 + heading2) / 2;
  heading_angle = 360 - heading_angle; // 회전 좌표계를 반시계 방향으로 할 것
  heading_angle_error = target_heading_angle - heading_angle;

  yaw1 = heading_angle; // IMU 측정값을 yaw1 변수에 저장

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
}

void imu_rotation(void)
{
  bool flag = true; // bool 타입은 true(1) 또는 false(0)
  while (flag)
  {
    read_imu_sensor();

    if (heading_angle_error > THRESHOLD_ANGLE1) // 반시계방향으로 회전
    {
      motor_control_L(-200);
      motor_control_R(200);
    }
    else if ((heading_angle_error >= THRESHOLD_ANGLE2) && (heading_angle_error <= THRESHOLD_ANGLE1)) //
    {
      motor_control_L(-200);
      motor_control_R(200);
    }
    else if ((heading_angle_error > -THRESHOLD_ANGLE2) && (heading_angle_error < THRESHOLD_ANGLE2)) // 정지
    {
      motor_control_L(0);
      motor_control_R(0);
      flag = false; // 루프를 빠져나오도록 설정
    }
    else if ((heading_angle_error >= -THRESHOLD_ANGLE1) && (heading_angle_error <= -THRESHOLD_ANGLE2)) //
    {
      motor_control_L(200);
      motor_control_R(-200);
    }
    else // heading_angle_error <-THRESHOLD_ANGLE // 시계방향으로 회전
    {
      motor_control_L(200);
      motor_control_R(-200);
    }
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
  }
}

////////////////////////////////// Encoder ////////////////////////////////
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

/////////////////////////// Waypoint ////////////////////////////
void init_waypoint(void)
{
  int no_waypoints = 9;
  // wp-0 line_follwing
  waypoint[0].pos_x = 1; waypoint[0].pos_y = 0;  waypoint[0].angle =   0; waypoint[0].control_type = VLC;
  // wp-1 yaw_control
  waypoint[1].pos_x = 1; waypoint[1].pos_y = 0;  waypoint[1].angle = -90; waypoint[1].control_type = IHC;
  // wp-2 line_follwing
  waypoint[2].pos_x = 1; waypoint[2].pos_y = -1; waypoint[2].angle = -90; waypoint[2].control_type = VLC;
  // wp-3 line_follwing
  waypoint[3].pos_x = 1; waypoint[3].pos_y = -2; waypoint[3].angle = -90; waypoint[3].control_type = VLC;
  // wp-4 yaw_control
  waypoint[4].pos_x = 1; waypoint[4].pos_y = -2; waypoint[4].angle =   0; waypoint[4].control_type = IHC;
  // wp-5 line_follwing
  waypoint[5].pos_x = 2; waypoint[5].pos_y = -2; waypoint[5].angle =   0; waypoint[5].control_type = VLC;
  // wp-6 line_follwing
  waypoint[6].pos_x = 3; waypoint[6].pos_y = -2; waypoint[6].angle =   0; waypoint[6].control_type = VLC;
  // wp-7 line_follwing
  waypoint[7].pos_x = 4; waypoint[7].pos_y = -2; waypoint[7].angle =   0; waypoint[7].control_type = VLC;
  // wp-8 line_follwing
  waypoint[8].pos_x = 5; waypoint[8].pos_y = -2; waypoint[8].angle =   0; waypoint[8].control_type = VLC;
}

void VLC_move(void)  // 비전 라인 컨트롤
{
  int cx_int;
  double cx = 0;
  
  cx_int = (int)cx;
  read_line_camera();
  threshold_line_image(70);  // 값 조정
  cx = line_centroid();
  vision_line_control( 80 , cx);

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

void IHC_move(double taget_yaw)
{
  //yaw 넣어
}

////////////////////// BlueTooth ////////////////////////////
void BlueTooth_control()
{
  if (BTSerial.available())
  {
    char command = BTSerial.read();
    if (command == '1')
    {
      // Move forward
      motor_control_L(200);
      motor_control_R(200);
    }
    else if (command == '2')
    {
      // Move backward
      motor_control_L(-200);
      motor_control_R(-200);
    }
    else if (command == '3')
    {
      // Turn right
      motor_control_L(255);
      motor_control_R(-255);
    }
    else if (command == '4')
    {
      // Turn left
      motor_control_L(-255);
      motor_control_R(255);
    }
    else
    {
      // Stop
      motor_control_L(0);
      motor_control_R(0);
    }
    Serial.print("command: ");
    Serial.println(command);

  }
}

void loop()
{
  BlueTooth_control();
  read_line_camera();
  threshold_line_image(30);
  
  for(int i = 0; i < NPIXELS; i++)
  {
    Serial.println(LineSensor_threshold_Data[i]);
    Serial.println(Pixel[i]);
  }
  VLC_move();


} 
