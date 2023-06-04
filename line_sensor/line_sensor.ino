/////////////////// Line Sensor //////////////////

int line_sensor_pin[5] = {34 , 35, 36, 37, 38};  //Line Sensor pin 배열을 선언
int line_sensor[5]     = { 0 , 0 , 0 , 0 ,  0};  //Line Sensor 값 배열을 선언

int read_line_sensor(void)
{
  int i, line_index;
  int sum = 0; // line sensor에서 불이 들어온 sensor 갯수
  for (i=0;i<5;i++)
  {
    line_sensor[i] = digitalRead(line_sensor_pin[i]); //흰색이면 1 검정색이면 0
    sum += line_sensor[i];
    Serial.print(line_sensor[i]);   Serial.print("  ");
  }
  Serial.print("sum   ");      Serial.println(sum);
  // line_sensor[0]   line_sensor[1]    line_sensor[2]    line_sensor[3]    line_sensor[4]
  ///      -4       -3      -2        -1      0        1        2         3       4      
  
  if(sum == 1)
  {
    if(line_sensor[0]==1)  line_index = -4;
    if(line_sensor[1]==1)  line_index = -2;
    if(line_sensor[2]==1)  line_index =  0;
    if(line_sensor[3]==1)  line_index =  2;
    if(line_sensor[4]==1)  line_index =  4;
  }
  else if(sum == 2)
  {
   if((line_sensor[0]==1) && (line_sensor[1]==1)) line_index = -3;
   if((line_sensor[1]==1) && (line_sensor[2]==1)) line_index = -1;
   if((line_sensor[2]==1) && (line_sensor[3]==1)) line_index =  1;
   if((line_sensor[3]==1) && (line_sensor[4]==1)) line_index =  3; 
  }
  else if(sum == 5)
  {
    line_index = -10;
  }
  else
  {
    
  }
  Serial.print("line_index  "); Serial.println(line_index);

  return line_index;
  
}

void line_following(int line_index)
{
  switch(line_index)
  {
    case -4: motor_L_control(-40);  motor_R_control(80); 
    break;
    case -3: motor_L_control(5);  motor_R_control(80);
    break;
    case -2: motor_L_control(35);  motor_R_control(80);
    break;
    case -1: motor_L_control(60);  motor_R_control(80);
    break;
    case  0: motor_L_control(80);  motor_R_control(80);
    break;
    case  1: motor_L_control(140);  motor_R_control(60);
    break;
    case  2: motor_L_control(150);  motor_R_control(60);
    break;
    case  3: motor_L_control(160);  motor_R_control(- 50);
    break;
    case  4: motor_L_control(170);  motor_R_control(-40);
    break;
    case -10: motor_L_control(0);  motor_R_control(0);
    break;
  }
}
/////////////////////// PID //////////////////////
double Kp=0.25, Ki=0.0, Kd=0;


///////////////////// Driving //////////////////////
float t_angle[4] = {0, 90, 180, 270};

///////////////////// IMU //////////////////////
#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

LSM303 compass;

float heading_angle      = 0.0;
float init_heading_angle = 0.0;   // 초기 방향
float target_heading_angle = 0.0;
float heading_angle_error = 0.0;  // error 값


//////////////////////////sonar//////////////////////////
#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE      400 //mm 단위
#define WALL_GAP_DISTANCE_HALF 200 //mm 단위
#define MOTOR_PWM_OFFSET 10

#define Front 0
#define Left  1
#define Right 2

#define TRIG1 2
#define ECHO1 3

#define TRIG2 4
#define ECHO2 5

#define TRIG3 6
#define ECHO3 7
NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(TRIG2, ECHO2, MAX_DISTANCE),
  NewPing(TRIG3, ECHO3, MAX_DISTANCE)


};

float front_sonar1  = 0.0;
float left_sonar1   = 0.0;
float right_sonar1  = 0.0;

float front_sonar2  = 0.0;
float left_sonar2   = 0.0;
float right_sonar2  = 0.0;

float front_sonar3  = 0.0;
float left_sonar3   = 0.0;
float right_sonar3  = 0.0;

float front_sonar  = 0.0;
float left_sonar   = 0.0;
float right_sonar  = 0.0;


/////////////////// L298 ////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

/////////////// Maze Status /////////////////////
int maze_status = 0;

void setup()
{
  // put your setup code here, to run once:
  int i;

  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENB, OUTPUT);

  for(i=0;i<5;i++)
  {
  pinMode (line_sensor_pin[i], INPUT);
  }
  Serial.begin(115200); // 통신속도를 115200으로 정의함

  Wire.begin();  // IMU initiallize
  compass.init();
  compass.enableDefault();

  compass.m_min = (LSM303::vector<int16_t>) {-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>) {+32767, +32767, +32767};

}

  void motor_R_control(int motor_speed_R) // 모터 A의 속도(speed)제어
  {
    if (motor_speed_R >= 0)
    {
      digitalWrite(IN1, HIGH);         //모터의 방향 제어
      digitalWrite(IN2, LOW);
      if(motor_speed_R>=255) motor_speed_R = 255;
      analogWrite(ENA, motor_speed_R); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      if(motor_speed_R<=-255) motor_speed_R = -255;
      analogWrite(ENA, -motor_speed_R);
    }
  }


  void motor_L_control(int motor_speed_L) // 모터 A의 속도(speed)제어
  {
    if (motor_speed_L >= 0)
    {
      digitalWrite(IN3, HIGH);         //모터의 방향 제어
      digitalWrite(IN4, LOW);
      if(motor_speed_L>=255) motor_speed_L = 255;
      analogWrite(ENB, motor_speed_L); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      if(motor_speed_L<=-255) motor_speed_L = -255;
      analogWrite(ENB, -motor_speed_L);
    }
  }

 void imu_rotation(void)
 {
   bool flag = 1; // bool type은 0,1
   while(flag)
   {
      read_imu_sensor();
    
      if (heading_angle_error > THRESHOLD_ANGLE1) { // 반시계 방향으로 회전
      motor_L_control(-80);
      motor_R_control(80);
    }
    else if ((heading_angle_error >= THRESHOLD_ANGLE2) && (heading_angle_error <= THRESHOLD_ANGLE1)) { // 정지
      motor_L_control(-80);
      motor_R_control(80);
    }
    else if ((heading_angle_error > -THRESHOLD_ANGLE2) && (heading_angle_error < THRESHOLD_ANGLE2)) { // 정지
      motor_L_control(0);
      motor_R_control(0);
      flag = 0;
    }
    else if ((heading_angle_error >= -THRESHOLD_ANGLE1) && (heading_angle_error <= -THRESHOLD_ANGLE2)) { // 정지
      motor_L_control(80);
      motor_R_control(-80);
    }
    else  // heading_angle_error < -THRESHOLD_ANGLE1 // 시계방향으로 회전
    {  
      motor_L_control(80);
      motor_R_control(-80);
    }
    
  }
}

  void read_sonar_sensor(void)
  {
    float front_sonar1 = sonar[Front].ping_cm() * 10;
    float left_sonar1  = sonar[Left].ping_cm() * 10;
    float right_sonar1 = sonar[Right].ping_cm() * 10;

    float front_sonar2 = sonar[Front].ping_cm() * 10;
    float left_sonar2  = sonar[Left].ping_cm() * 10;
    float right_sonar2 = sonar[Right].ping_cm() * 10;

    float front_sonar3 = sonar[Front].ping_cm() * 10;
    float left_sonar3  = sonar[Left].ping_cm() * 10;
    float right_sonar3 = sonar[Right].ping_cm() * 10;

    front_sonar = (front_sonar1 + front_sonar2 + front_sonar3) / 3;
    left_sonar  = (left_sonar1 + left_sonar2 + left_sonar3) / 3;
    right_sonar = (right_sonar1 + right_sonar2 + right_sonar3) / 3;

    if (front_sonar == 0.0) front_sonar = MAX_DISTANCE * 10; // 0.0 출력이 최댓값이므로
    if (left_sonar  == 0.0) left_sonar = MAX_DISTANCE * 10;
    if (right_sonar == 0.0) right_sonar = MAX_DISTANCE * 10;

  /*      
    Serial.print("L:"); Serial.print(left_sonar); Serial.print(" ");
    Serial.print("F:"); Serial.print(front_sonar); Serial.print(" ");
    Serial.print("R:"); Serial.println(right_sonar);
  */
  }

  void read_imu_sensor(void)
  {
     compass.read();
     float heading1 = compass.heading();
     compass.read();
     float heading2 = compass.heading();
     heading_angle = (heading1 + heading2) / 2;
     heading_angle = 360 - heading_angle; // 회전 좌표계를 반시계 방향으로 할 것     
     heading_angle_error = target_heading_angle - heading_angle;

     if(heading_angle_error > 180)
     {
         heading_angle_error = heading_angle_error - 360;
     }
     else if(heading_angle_error < -180)
     {
         heading_angle_error = heading_angle_error + 360;  
     }
     else
     {


     }
     
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
  }
  void run_heading_angle(void)
  {
     bool flag = 1;
     double Output;
    
     while(flag)
     {
       read_sonar_sensor();
       read_imu_sensor();
       Output = Kp * heading_angle_error;

       motor_L_control(90 + Output); 
       motor_R_control(90 - Output);

       if(front_sonar < 170) 
       {
         flag = 0;
         motor_L_control(0); 
         motor_R_control(0);
       }
     }
     
  }
  void loop()
  {
    read_line_sensor();
    line_following(read_line_sensor());
  }
  void loop1()
  {
    /*target_heading_angle = 51;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = 27;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = 155;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = -275;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = 74;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = 2;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = -292;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = -344;
    imu_rotation();
    run_heading_angle();
    target_heading_angle = 16;
    imu_rotation();
    run_heading_angle();
    
    target_heading_angle = -358;
    imu_rotation();
    run_heading_angle();*/
  }
