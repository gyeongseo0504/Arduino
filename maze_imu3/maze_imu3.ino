/////////////////////// PID //////////////////////
double Kp=1.7, Ki=0.0, Kd=0;


///////////////////// Driving //////////////////////
float t_angle[4] = {0, 90, 180, 270};

///////////////////// IMU //////////////////////
#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE1 15
#define THRESHOLD_ANGLE2 7

LSM303 compass;

float heading_angle      = 0.0;
float init_heading_angle = 17.0;   // 초기 방향
float target_heading_angle = 90;
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

  pinMode (ENA, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);

  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENB, OUTPUT);


  Serial.begin(115200); // 통신속도를 115200으로 정의함

  Wire.begin();  // IMU initiallize
  compass.init();
  compass.enableDefault();

  /*
    Calibration values; the default values of +/-32767 for each axis
    lead to an assumed magnetometer bias of 0. Use the Calibrate example
    program to determine appropriate values for your particular unit.
  */
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
      digitalWrite(IN3, LOW);         //모터의 방향 제어
      digitalWrite(IN4, HIGH);
      if(motor_speed_L>=255) motor_speed_L = 255;
      analogWrite(ENB, motor_speed_L); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      if(motor_speed_L<=-255) motor_speed_L = -255;
      analogWrite(ENB, -motor_speed_L);
    }
  }



  void check_maze_status(void)
  {
    if ( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF  ) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF  ) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF  ))

    {
      maze_status = 4;
      Serial.println("maze_status = 4");
    }
    else if ( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF  ) && (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF  ) && (front_sonar >= WALL_GAP_DISTANCE_HALF  ) )
    {
      maze_status = 1;
      Serial.println("maze_status = 1");
    }
    else if ( (left_sonar >= 0) && (left_sonar <= WALL_GAP_DISTANCE_HALF  ) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF  ) )
    {
      maze_status = 2;
      Serial.println("maze_status = 2");
    }
    else if ( (right_sonar >= 0) && (right_sonar <= WALL_GAP_DISTANCE_HALF  ) && (front_sonar >= 0) && (front_sonar <= WALL_GAP_DISTANCE_HALF  ) )
    {
      maze_status = 3;
      Serial.println("maze_status = 3");
    }
    else
    {
      maze_status = 0;
      Serial.println("maze_status = 0");
    }
  }

  //먼저 left_pwm =0;  right_pwm =100; 으로 해서 왼쪽 오른쪽 방향 찾기
  void wall_collision_avoid(int base_speed)
  {
    float error = 0.0;
    float Kp = 0.26;       //나중에 조정해야 할 것
    int pwm_conrol = 0; // pwm을 0으로 초기화
    int right_pwm = 0;  // 모터속도 0으로 초기화
    int left_pwm = 0;   // 모터속도 0으로 초기화

    error = (right_sonar - left_sonar);
    error = Kp * error;

    if (error >=  40) error =  40;
    if (error <= -40) error = -40;

    right_pwm = base_speed - error;
    left_pwm  = base_speed + error * 1.5 ;

    if (right_pwm <= 0) right_pwm = 0;
    if (left_pwm  <= 0) left_pwm  = 0;

    if (right_pwm >= 120) right_pwm = 120;
    if (left_pwm  >= 120) left_pwm  = 120;
/*
    motor_A_control(HIGH, right_pwm); // 오른쪽 전진
    motor_B_control(HIGH, left_pwm); // 왼쪽 전진
*/
  }

 void imu_rotation(void)
 {
   bool flag = 1; // bool type은 0,1
   while(flag)
   {
      read_imu_sensor();
    
      if(heading_angle_error > THRESHOLD_ANGLE1) // 반시계방향으로 회전
      {
          motor_L_control(-70); 
          motor_R_control(60);
      }
      else if((heading_angle_error >=THRESHOLD_ANGLE2) && (heading_angle_error <=THRESHOLD_ANGLE1)) // 
      {
        motor_L_control(-40); 
        motor_R_control(40);
      }
      else if((heading_angle_error >-THRESHOLD_ANGLE2) && (heading_angle_error <THRESHOLD_ANGLE2)) // 정지
      {
        motor_L_control(0); 
        motor_R_control(0);
        flag = 0;
      }
      else if((heading_angle_error >=-THRESHOLD_ANGLE1) && (heading_angle_error <=-THRESHOLD_ANGLE2)) // 
      {
        motor_L_control(70); 
        motor_R_control(-70);
      }
      else // heading_angle_error <-THRESHOLD_ANGLE // 시계방향으로 회전
      {
        motor_L_control(70); 
        motor_R_control(-70);
      }
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
    
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

  }

  void read_imu_sensor(void)
  {
     double Output;
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

       motor_L_control(70 - Output ); 
       motor_R_control(70 + Output);

       if(front_sonar < 200) 
       {
         flag = 0;
         motor_L_control(0); 
         motor_R_control(0);
       }
     }
     
  }
  
  void loop(){
    // put your main code here, to run repeatedly:

  
/*
    target_heading_angle = 130;
    imu_rotation();
    delay(2000); // 최종적으로 딜레이는 빼지만 확인하기 위해서 넣는다
    target_heading_angle = 90;
    imu_rotation();
    delay(2000);
    */

    target_heading_angle = 130;
    run_heading_angle();
    delay(2000);
    target_heading_angle = 130 - 100;
    imu_rotation();
    delay(2000);
    run_heading_angle();
    
    Serial.print("L:"); Serial.print(left_sonar); Serial.print(" ");
    Serial.print("F:"); Serial.print(front_sonar); Serial.print(" ");
    Serial.print("R:"); Serial.println(right_sonar);
    Serial.print("Heading Angle Error : ");
    Serial.print(heading_angle_error); //heading_angle error 표시
    Serial.print(" = ");
    Serial.print(target_heading_angle);
    Serial.print(" - ");
    Serial.println(heading_angle); //heading_angle 표시
    
    check_maze_status();
  }
  
  /*
     if(maze_status == 4) // 왼 가 오
    {
      Serial.println("Rotate CCW");
      motor_A_control(HIGH,130); // 오른쪽 전진
      motor_B_control(LOW,130); // 오른쪽 후진
      delay(930);
    }
    else if( maze_status == 1) // 왼 오
    {
      Serial.println("run straight");
      wall_collision_avoid(70);
    }

     else if( maze_status == 2) //왼 가
    {
      Serial.println("Rotate CCW");
      motor_A_control(LOW,100);
      motor_B_control(HIGH,100);
      delay(560);

    }
     else if ( maze_status == 3) //가 오
    {
      Serial.println("Rotate CCW");
      motor_A_control(HIGH,100);
      motor_B_control(LOW,100);
      delay(540);
    }
     else
    {
      Serial.println("Go straight");
      motor_A_control(HIGH, 70);
      motor_B_control(HIGH, 70);
    }
  */
