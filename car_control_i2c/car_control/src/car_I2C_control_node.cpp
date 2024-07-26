/*******************************************************************************
 *  Functions
 *******************************************************************************
 */
/*******************************************************************************
 *  INCLUDE #define POLYNORMIAL 0xA001FILES
 *******************************************************************************
 */
#include <sys/ioctl.h>
#include "ros/ros.h" 
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include <geometry_msgs/Vector3.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h> 
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <termios.h>                   // B115200, CS8 등 상수 정의
#include <fcntl.h>                     // O_RDWR , O_NOCTTY 등의 상수 정의
#include <time.h>
#include <math.h>
#include <pthread.h>

/*******************************************************************************
 *  Defines
 *******************************************************************************
 */

//i2c address  
#define ADDRESS 0x05

//I2C bus  
static const char *deviceName = "/dev/i2c-0";

#define _USE_MATH_DEFINES

#define MAX_MOTOR_SPEED_RPM  150
#define Neutral_Steer_angle   0
#define LOOP_RATE            20


/*****************************
* Robot STEER Control define 
******************************/
#define MAX_R_ANGLE -40
#define MAX_L_ANGLE 40

/*****************************
* Robot SPEED Control define 
******************************/
#define MAX_ROBOT_SPEED  250
#define MIN_ROBOT_SPEED  -250 



/*****************************
* Robot parameter define 
******************************/
#define ROBOT_WHEEL_BASE


typedef unsigned char BYTE;


#define DEG2RAD(x) (M_PI/180.0*(x) )
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define RPM2RPS(x) ((x)/60) 
#define RPS2RPM(x) ((x)*60) 

union
{
    float data ;
    char  bytedata[4];
    
} m_robot_speed_float , m_current_robot_speed_float ,m_distance_float;

union
{
    short data ;
    char  bytedata[2];
    
} m_robot_angle_int16 , m_current_robot_speed_int16;

double m_acceleration =  500;  //50pwm/s
double m_deceleration =  500;  //50pwm/s


static int uart_fd;
unsigned char protocal_test[16] ={0,};
unsigned char received_data[7]  ={0,};

double speed_factor = 255;
double steer_factor = 20;

int file_I2C;
int speed_stop = 0;
int sonar_flag = 0;

int open_I2C(void)
{
   int file;  
   
    if ((file = open( deviceName, O_RDWR ) ) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);  
        exit(1);  
    }  
    printf("I2C: Connected\n");  
  
   
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);  
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);  
        exit(1);  
    } 
    
    return file; 
}

/*
설명 : I2C포트를 닫는다.
*/

void close_I2C(int fd)
{
   close(fd);
}

void sonar_callback(const std_msgs::Int8 & msg)
{
	sonar_flag = msg.data;
}
	
	
void cmd_callback(const geometry_msgs::Twist & cmd_input)
{
    float angular_temp;
    float linear_temp;
    
    linear_temp  = cmd_input.linear.x ;//RPM
    angular_temp = cmd_input.angular.z ;//rad/s

    
    if(angular_temp <= MAX_R_ANGLE)  angular_temp = MAX_R_ANGLE;
    if(angular_temp >= MAX_L_ANGLE)  angular_temp = MAX_L_ANGLE;   
    m_robot_angle_int16.data  = (short)angular_temp+ Neutral_Steer_angle; 
       
    
    if(linear_temp >=  MAX_ROBOT_SPEED)    linear_temp = MAX_ROBOT_SPEED;
    if(linear_temp <=  MIN_ROBOT_SPEED)    linear_temp = MIN_ROBOT_SPEED;
    m_robot_speed_float.data = linear_temp;
    
}


void boat_cmd_callback(const geometry_msgs::Twist & cmd_input)
{
    float angular_temp;
    float linear_temp;
    float r =0;
    
    linear_temp  = cmd_input.linear.x * speed_factor;     //  1m/s 일때 255 pwm 출력 계수 나중에 조정할것
    angular_temp = cmd_input.angular.z ;         //  각속도 v= r x w 
    r = linear_temp/angular_temp;
    r = steer_factor/r;                                    //  r 회전 반경이 작을 수록 방향타는 크게 해야 함 계수조정할 것 
     
    if(angular_temp <= MAX_R_ANGLE)  angular_temp = MAX_R_ANGLE;
    if(angular_temp >= MAX_L_ANGLE)  angular_temp = MAX_L_ANGLE;   
    m_robot_angle_int16.data  = (short)angular_temp+ Neutral_Steer_angle; 
       
    
    if(linear_temp >=  MAX_ROBOT_SPEED)    linear_temp = MAX_ROBOT_SPEED;
    if(linear_temp <=  MIN_ROBOT_SPEED)    linear_temp = MIN_ROBOT_SPEED;
    m_robot_speed_float.data = linear_temp;
         
}

void wp_speed_callback(const std_msgs::Int16 & speed_input)
{
    int speed_temp;
    
    speed_temp = speed_input.data;//m/s
    
    speed_stop = speed_temp;
    
    if(speed_temp >=  MAX_ROBOT_SPEED)    speed_temp = MAX_ROBOT_SPEED;
    if(speed_temp <=  MIN_ROBOT_SPEED)    speed_temp = MIN_ROBOT_SPEED;
    m_robot_speed_float.data = speed_temp;            
}

void wp_steer_callback(const std_msgs::Int16 & steer_input)
{
    int steer_temp;
    
    steer_temp = steer_input.data;//rad/s

    
    if(steer_temp <= MAX_R_ANGLE)  steer_temp = MAX_R_ANGLE;
    if(steer_temp >= MAX_L_ANGLE)  steer_temp = MAX_L_ANGLE;   
    m_robot_angle_int16.data  = (short)steer_temp+ Neutral_Steer_angle; 
              
}


void robot_speed_profile_control(void)
{
  
  if(m_current_robot_speed_float.data > m_robot_speed_float.data)   //현재 속도가 명령 속도 보다 클 때 , 감속 조건  m_robot_speed 가 명령어임
  {
	 
	 m_current_robot_speed_float.data -=  m_deceleration / LOOP_RATE;
	 
	 m_current_robot_speed_float.data = (m_current_robot_speed_float.data <= m_robot_speed_float.data ) ? m_robot_speed_float.data : m_current_robot_speed_float.data;
	  
  }
  else if(m_current_robot_speed_int16.data < m_robot_speed_float.data) //현재 속도가 명령속도 보다 클때 , 가속 조건
  {
	  
	 m_current_robot_speed_float.data +=  m_acceleration / LOOP_RATE; 
	 
	 m_current_robot_speed_float.data = (m_current_robot_speed_float.data >= m_robot_speed_float.data ) ? m_robot_speed_float.data : m_current_robot_speed_float.data;
  }
  else 
  {
	   m_current_robot_speed_float.data = m_robot_speed_float.data;
  }   
  
}



void *read_I2C_thread(void *pt)
{
		int num_bytes = -1;
		unsigned char insert_buf;
		
		while(1){
			while(num_bytes = read(file_I2C, received_data, 6) > 0){
				if((received_data[0] == 'D') && (received_data[5] == '*'))
				m_distance_float.bytedata[0] = received_data[1];
				m_distance_float.bytedata[1] = received_data[2];
				m_distance_float.bytedata[2] = received_data[3];
				m_distance_float.bytedata[3] = received_data[4];
				}
			sleep(100);
			}
}




int main(int argc, char **argv)
{
	
  file_I2C = open_I2C();
  if(file_I2C < 0)
  {
	  printf("Unable to open I2C\n");
	  return -1;
  }
  else
  {
	  printf("I2C is Connected\n");
  }
  
  
  ros::init(argc, argv, "car_I2C_control_node");
  ros::NodeHandle n;

  std::string cmd_vel_topic = "cmd_vel";
  std::string boat_cmd_vel_topic = "/boat_cmd_vel";
  std::string cmd_vel_topic2 = "Car_Control_cmd/Speed_Int16";
  std::string cmd_vel_topic3 = "Car_Control_Cmd/steerAngle_Int16";
  std::string odom_pub_topic = "odom";
  //std::string wheel_speed_pub_topic = "wheel_speed";
  //std::string wheel_encoder_pub_topic = "wheel_encoder";
  
  std::string odom_frame_id       = "odom";  
  std::string odom_child_frame_id = "base_footprint";
  std::string odom_topic          = "/odom/distance"; 
  std::string sonar_topic         = "flag/sonar";
  //pthread_t id_1;
  //int ret1 = pthread_create(&id_1,NULL,*read_I2C_thread,NULL);
  
  
  std_msgs::Float32 odom_data_msg;
  
  ros::param::get("accleration",  m_acceleration); //acceleraton  parameter
  ros::param::get("deceleration", m_deceleration); //deceleration parameter
  ros::param::get("speed_factor",speed_factor);
  ros::param::get("steer_factor",steer_factor);
  
  
  ros::Subscriber sub1 = n.subscribe(cmd_vel_topic, 20, cmd_callback);
  ros::Subscriber sub2 = n.subscribe(boat_cmd_vel_topic, 20,boat_cmd_callback);
  ros::Subscriber sub_wp_speed = n.subscribe(cmd_vel_topic2, 20, wp_speed_callback);
  ros::Subscriber sub3_wp_angle = n.subscribe(cmd_vel_topic3, 20, wp_steer_callback);
  ros::Subscriber sub_sonar = n.subscribe(sonar_topic, 20, sonar_callback);
  
  ros::Publisher  odom_pub      = n.advertise<std_msgs::Float32>(odom_topic, 1);
  
  ros::Rate loop_rate(LOOP_RATE); //10.0HZ
  
  m_robot_speed_float.data = 0.0;
  m_robot_angle_int16.data = 0;
  
  while(ros::ok())
  {
	 odom_data_msg.data = m_distance_float.data;
	 odom_pub.publish(odom_data_msg);
	 //printf("test\n");
     //printf("S:%4.2lf | A:%3d\n", m_current_robot_speed_float.data, m_robot_angle_int16.data);
    robot_speed_profile_control(); // 로봇의 가감속 속도 제어를 함    
     
	protocal_test[0] = '#';
	protocal_test[1] = 'C';
    //m_robot_angle_int16.data = (float)m_robot_angle_int16.data*1.3;
    if(m_robot_angle_int16.data >0) (m_robot_angle_int16.data + 10);
	protocal_test[2] = m_robot_angle_int16.bytedata[0];
	protocal_test[3] = m_robot_angle_int16.bytedata[1];
    //m_robot_angle_int16.data -= 8;
    if(speed_stop == 0) m_current_robot_speed_float.data = 0;
	protocal_test[4] = m_current_robot_speed_float.bytedata[0];
	protocal_test[5] = m_current_robot_speed_float.bytedata[1];  
	protocal_test[6] = m_current_robot_speed_float.bytedata[2];  
	protocal_test[7] = m_current_robot_speed_float.bytedata[3];    
	protocal_test[8] = '*';
	protocal_test[9] = sonar_flag;
     
	write(file_I2C, protocal_test, 10); 
	
	
	read(file_I2C, received_data, 6);
	if((received_data[0] == 'D') && (received_data[5] == '*')){
				m_distance_float.bytedata[0] = received_data[1];
				m_distance_float.bytedata[1] = received_data[2];
				m_distance_float.bytedata[2] = received_data[3];
				m_distance_float.bytedata[3] = received_data[4];}
	
	printf("speed : %lf  angle : %d distance : %lf flag : %d\n",m_current_robot_speed_float.data,m_robot_angle_int16.data,m_distance_float.data,sonar_flag);
	
	ros::spinOnce();
	loop_rate.sleep();
  }

  return 0;
}
