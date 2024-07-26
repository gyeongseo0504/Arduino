#include "ros/ros.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#define YAW_CONTROL           0
#define LANE_CONTROL          1
#define CONE_TRAFFIC_CONTROL  2
#define OVERTAKE_CONTROL      3 
#define MAZE_CONTROL          4
#define MAZE_TURN_90          5

// maximum angular velocity 
#define MAX_L_STEER -28.0
#define MAX_R_STEER  30.0

#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

#define WayPoints_NO 6
#define WayPoint_X_Tor 0.4
#define WayPoint_Y_Tor 0.4

#define SpeedRegion_NO 100

#define rotary_location  0

#define LEFT            -1
#define CENTER           0
#define RIGHT            1

double pos_x = 0.0;
double pos_y = 0.0;
double roll,pitch,yaw;

double r,p,y;


double imu1_roll,imu1_pitch,imu1_yaw;
double imu1_heading_angle_radian;

int    car_speed_base         = 180; //[pwm]
int    steer_car_speed_max    = 255;
int    car_steer_angle        = 0;
int    max_steer_angle        = 30;

double car_yaw_angle          = 0.0;
double car_yaw_angle_d        = 0.0;
float  odom_distance          = 0.0;

int    sonar;
int    traffic_data           = 0;
int    mission_flag           = 0;
int    wall_count 			  = 0;
bool   stop_sign_flag         = false;
double car_linear_odom        = 0.0;   // unit[m]
double car_linear_maze        = 3.0;   // unit[m]
int    detect_traffic_sign    = CENTER;
double target_yaw             = 0.0;
double front_obstacle_distance = 0.0;


#define RAD2DEG(x) ((x)*180./M_PI)
#define DEG2RAD(x) ((x)/180.*M_PI)

std_msgs::Float32      target_yaw_msg;
std_msgs::Float32      target_overtake_lane_x_msg;

std::string imu1_topic                             = "/handsfree/imu/data";
std::string imu1_yaw_degree_topic                  = "/handsfree/imu/yaw_degree";
std::string imu1_yaw_radian_topic                  = "/handsfree/imu/yaw_radian";
std_msgs::Float32 imu_heading_angle_msg;

std::string imu_heading_angle_radian_topic         = "/imu/heading_angle_radian";
std::string imu_heading_angle_degree_topic         = "/imu/heading_angle_degree";
std::string imu_heading_angle_offset_degree_topic  = "/imu/heading_angle_offset_degree";

std::string imu_correction_enable__topic           = "/flag/imu_auto_correction";


struct Point 
{ 
	float x; 
	float y; 
	float z;
};


struct WayPoints
{
	float x;
	float y;
	
} ;

struct Current_Pos
{
	float x = 0.0;
	float y = 0.0;
	float theta = 0.0 ;
} my_pose;

struct region
{
	float left;
	float right;
	float top;
	float bottom;
} speed_region[SpeedRegion_NO];


void imu1Callback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);      
 
    m.getRPY(imu1_roll, imu1_pitch, imu1_yaw);
}

void imu1yawradianCallback(const std_msgs::Float32& msg)
{
	imu1_heading_angle_radian = msg.data;
	if(imu1_heading_angle_radian >= M_PI)  imu1_heading_angle_radian = 2*M_PI - imu1_heading_angle_radian;
    ROS_INFO("Received imu1_heading_angle_radian: %f", imu1_heading_angle_radian);

}


void sonar_Callback(const std_msgs::Int8 & msg)
{
	sonar = msg.data;
	printf("sonar flag : %d\n", sonar);
}

void odom_distance_Callback(const std_msgs::Float32 & msg)
{
	car_linear_odom = msg.data;
}
void maze_distance_Callback(const std_msgs::Float32 & msg)
{
	car_linear_maze = msg.data;
}
void front_obstacle_distance_Callback(const std_msgs::Float32 & msg)
{
	front_obstacle_distance = msg.data;
}

void trafficCallback(const std_msgs::Int8& msg)
{
	traffic_data = msg.data;
}

void car_control_steer_Callback(const std_msgs::Int16& msg)
{
	car_steer_angle = msg.data;
}

void stop_line_detect_Callback(const std_msgs::Bool& msg)
{
	stop_sign_flag      = msg.data;	
	
}

void traffic_sign_detect_Callback(const std_msgs::Int8& msg)
{
	detect_traffic_sign = msg.data;	
}

int corner_speed_control(int steer_angle, int base_speed, int max_corner_speed)
{
	int speed;
	
	double a, b;
	
	a =(max_corner_speed - base_speed)/(double)max_steer_angle;
	
	b = (double)base_speed;
	
	speed = (int)(a * fabs(steer_angle)) + base_speed;
	
	speed  = (speed >= max_corner_speed) ? max_corner_speed : speed;
	speed  = (speed <=                0) ?                0 : speed;
	
	return speed;
}


int main(int argc, char **argv)
{
	int count = 0;

	ros::init(argc, argv, "aa10_navigation_control");

	ros::NodeHandle n;

	std::string imu1_topic                             = "/handsfree/imu/data";
	std::string imu_yaw_angle_topic                = "/handsfree/imu/yaw_radian";
	std::string yaw_control_mode_topic             = "/Car_Control_Cmd/steering_control_mode";
	std::string imu_heading_angle_radian_topic         = "/imu/heading_angle_radian";
	std::string imu_heading_angle_degree_topic         = "/imu/heading_angle_degree";
	std::string imu_heading_angle_offset_degree_topic  = "/imu/heading_angle_offset_degree";
	std::string traffic_light_topic                    ="/vision/traffic_light";
	std::string sonar_flag_topic                       ="/flag/sonar";
	
	std::string odom_sub_topic 				       = "/odom";                                    // receive odom or pose
	std::string yaw_control_steering_output_topic  = "/Car_Control_Cmd/steerAngle_Int16";
	std::string car_control_speed_topic            = "/Car_Control_cmd/Speed_Int16";
	std::string lane_control_topic                 = "/flag/lane_control_set";
	std::string maze_control_topic                 = "/flag/maze_control_set";
	std::string traffic_sign_detect_flag_topic     = "/flag/traffic_sign_detect_set";
	std::string odom_distance_topic                = "/odom/distance";
	std::string maze_distance_topic                = "/lidar/front_obstacle_distance";
	std::string maze_flag_topic                    = "/flag/maze_control_set";
	std::string stop_line_detect_topic             = "/vision/stop_line_detect_flag";
	std::string target_overtake_lane_x_topic       ="/target/overtake_lane_x";

	std::string Overtake_mode                      ="/flag/overtake_mode";
	std::string Overtake_lane                      ="/xte/overtake_lane";
	
	
	ros::param::get("~imu_yaw_angle_topic",                   imu_yaw_angle_topic);     	
	ros::param::get("~yaw_control_mode_topic",                yaw_control_mode_topic);
	
	ros::param::get("~odom_sub_topic",                        odom_sub_topic);
	ros::param::get("~yaw_control_steering_output_topic",     yaw_control_steering_output_topic);
	ros::param::get("~stop_line_detect_topic"           ,     stop_line_detect_topic);
	
	ros::param::get("~yaw_control_steering_output_topic",     yaw_control_steering_output_topic);
	
	
	ros::Subscriber sub_obstacle_detect_distance        = n.subscribe("/lidar/front_obstacel_distance", 1, front_obstacle_distance_Callback);
	ros::Subscriber sub_car_control_steer               = n.subscribe(yaw_control_steering_output_topic, 1, car_control_steer_Callback);
	ros::Subscriber sub_stop_line_detect                = n.subscribe(stop_line_detect_topic , 1, stop_line_detect_Callback);
	ros::Subscriber sub_odom_distance                   = n.subscribe(odom_distance_topic , 1, odom_distance_Callback);
	ros::Subscriber sub_maze_distance                   = n.subscribe(maze_distance_topic , 1, maze_distance_Callback);
	ros::Subscriber sub_imu1                     		= n.subscribe("/handsfree/imu/yaw_degree",1,&imu1Callback);
	ros::Subscriber sub_imu1_yaw_radian        		    = n.subscribe("/handsfree/imu/yaw_radian",1,&imu1yawradianCallback);
	ros::Subscriber sub_traffic_light                   = n.subscribe(traffic_light_topic, 1, &trafficCallback);
			
	ros::Publisher  traffic_sign_detect_flag_pub        = n.advertise<std_msgs::Bool>(traffic_sign_detect_flag_topic, 1);
	ros::Publisher  lane_control_topic_pub              = n.advertise<std_msgs::Bool>(lane_control_topic, 1);
	ros::Publisher  overtake_flag_pub                   = n.advertise<std_msgs::Bool>(Overtake_mode,1);
	ros::Publisher  yaw_control_mode_pub                = n.advertise<std_msgs::Int8>(yaw_control_mode_topic, 1);
	ros::Publisher  maze_control_topic_pub              = n.advertise<std_msgs::Bool>(maze_control_topic, 1);
	ros::Publisher  car_control_speed_topic_pub         = n.advertise<std_msgs::Int16>(car_control_speed_topic, 1);
	ros::Publisher  target_yaw_pub                      = n.advertise<std_msgs::Float32>("/Car_Control_Cmd/Target_Angle", 1);
    ros::Publisher  imu_heading_angle_radian_pub        = n.advertise<std_msgs::Float32>(imu_heading_angle_radian_topic, 1);
	ros::Publisher  overtake_line_pub                   = n.advertise<std_msgs::Float32>(Overtake_lane,1);
	ros::Publisher  maze_flag_pub                       = n.advertise<std_msgs::Bool>(maze_flag_topic, 1);
	ros::Publisher  sonar_flag_pub                      = n.advertise<std_msgs::Int8>(sonar_flag_topic, 1);
	ros::Publisher  overtake_lane_control_pub           = n.advertise<std_msgs::Float32>(target_overtake_lane_x_topic,1);
	
    std_msgs::Float32 headangle_msg;
    std_msgs::Bool    lane_control_flag_msg;
    std_msgs::Int16   car_control_speed_msg;
    std_msgs::Int8    yaw_control_mode_msg;
    std_msgs::Bool	  traffic_sign_detect_msg;
	std_msgs::Bool    overtake_flag_msg;
	std_msgs::Float32 overtake_lane_msg;
	std_msgs::Bool    maze_flag_msg;
	std_msgs::Int8    sonar_flag_msg;
	std_msgs::Bool  maze_control_flag_msg;

	ros::Rate loop_rate(30);  // 10
	int mission_flag=0;
	double obs_distance=0;
	ros::param::get("~mission_flag",                   mission_flag);  
	ros::param::get("~obs_distance",                   obs_distance);  
	
	double car_linear_odom_temp   = 0.0;   // unit[m]
	double car_yaw_angle_d_temp   = 0.0;
	double car_move_distance      = 0.0;   // unit[m] 



	while (ros::ok())
	{
				boost::shared_ptr<sensor_msgs::Imu const> shared_imu1_topic, shared_imu2_topic;

		ROS_INFO("Mission flag : %2d | Speed :%3d   | Steer :%3d", mission_flag,car_control_speed_msg.data,car_steer_angle );
		switch(mission_flag)
		{
			case 0 :  // vision control //
					sonar_flag_msg.data = 0;
					sonar_flag_pub.publish(sonar_flag_msg);
			
					lane_control_flag_msg.data = true;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					
					traffic_sign_detect_msg.data = false;
					traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);

					overtake_flag_msg.data = false;
					overtake_flag_pub.publish(overtake_flag_msg);
					
					yaw_control_mode_msg.data = LANE_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);
					
					//traffic_data
				    ROS_INFO("Mission Lane Detection");
				    ROS_INFO("Traffic_data : %d",traffic_data);

				    car_control_speed_msg.data = 200;
				    car_control_speed_topic_pub.publish(car_control_speed_msg);
				    
				    if(stop_sign_flag == true)
					{
						ROS_INFO("Mission First Stop Detection");
						
						car_control_speed_msg.data = 0;
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						ros::Duration(2.0).sleep();
						traffic_sign_detect_msg.data = true;
						traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);
						ros::Duration(1.0).sleep();
						mission_flag++;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					break;
												
			case 1 : // wait for traffic sign detection result
					ROS_INFO("Traffic Sign Detection %d",detect_traffic_sign);
					
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);	
					
					traffic_sign_detect_msg.data = true;
					traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);				
					
					ros::Duration(1.5).sleep();
					detect_traffic_sign = RIGHT;   //not working
					if(detect_traffic_sign == CENTER)//(traffic_data == 2)
					{
						mission_flag = 2;
					}
					else if(detect_traffic_sign == RIGHT)//(traffic_data == 3)
					{
						mission_flag = 3;
					}
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					// car start
					car_control_speed_msg.data = 0;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////					

			case 2 : // straight yaw control   원형 구간 직진 1차 yaw_control로 직진
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					traffic_sign_detect_msg.data = false;
					traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);

					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					target_yaw = car_yaw_angle_d_temp;
					target_yaw_msg.data = target_yaw-5;
					target_yaw_pub.publish(target_yaw_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					
					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					 
					if(car_move_distance >= 3.2)
					{
						mission_flag = 4;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}
					break;		
					
			case 3 : // right turn    원형 구간 우회전 1차 yaw_control로 직진
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					traffic_sign_detect_msg.data = false;
					traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					
					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					
					target_yaw = car_yaw_angle_d_temp;			
					target_yaw_msg.data = target_yaw-5;
					target_yaw_pub.publish(target_yaw_msg);

					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);
						
					printf("distance: %6.3lf\n, car_linear_odom");

					if(car_move_distance >= 1.0){
						mission_flag = 6;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}
					break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			case 4 : // vision control 직진  원형 구간 직진 2차 lane_control로 이동
					lane_control_flag_msg.data = true;
					lane_control_topic_pub.publish(lane_control_flag_msg);

					yaw_control_mode_msg.data = LANE_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					if(car_move_distance >= 12.7)//stop_sign_flag == true
					{
						mission_flag = 5;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}		
					break;



			case 5 : // yaw control 직진  원형 구간 직진 3차 yaw_control로 이동 (원형구간 직진 끝)
					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					target_yaw = car_yaw_angle_d_temp  + 40;
					target_yaw_msg.data = target_yaw;
					target_yaw_pub.publish(target_yaw_msg);

					if(car_move_distance >= 2.5){
						mission_flag = 9;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						}	
					break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			case 6 : // right turn    원형 구간 우회전 1차 yaw_control로 우회전
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					traffic_sign_detect_msg.data = false;
					traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					
					car_move_distance =  car_linear_odom - car_linear_odom_temp;

					target_yaw = car_yaw_angle_d_temp -90;
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);					
					target_yaw_msg.data = target_yaw;
					target_yaw_pub.publish(target_yaw_msg);
					printf("distance: %6.3lf\n, car_linear_odom");
					
					if(car_move_distance >= 2.2)
					{
						printf("distance: %6.3lf\n, car_linear_odom");
						mission_flag = 7;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}
					break;	

			case 7 : // vision control 직진  원형 구간 우회전 2차 lane_control로 이동
					lane_control_flag_msg.data = true;
					lane_control_topic_pub.publish(lane_control_flag_msg);

					yaw_control_mode_msg.data = LANE_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					if(car_move_distance >= 6.15)//stop_sign_flag == true
					{
						mission_flag = 8;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}		
					break;

			case 8 : // right turn    원형 구간 우회전 3차 yaw_control로 우회전 (원형구간 우회전 끝)
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					traffic_sign_detect_msg.data = false;
					traffic_sign_detect_flag_pub.publish(traffic_sign_detect_msg);

					car_control_speed_msg.data = 200;                                       // stop
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					
					car_move_distance =  car_linear_odom - car_linear_odom_temp;

					target_yaw = car_yaw_angle_d_temp -65;
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);					
					target_yaw_msg.data = target_yaw;
					target_yaw_pub.publish(target_yaw_msg);
					printf("distance: %6.3lf\n, car_linear_odom");
					
					if(car_move_distance >= 2.6)
					{
						printf("distance: %6.3lf\n, car_linear_odom");
						mission_flag = 9;
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					}
					break;	

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
					
			case 9: //긴 라인주행 및 장애물 정지
				    lane_control_flag_msg.data = true;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					yaw_control_mode_msg.data = LANE_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);
					
					maze_flag_msg.data = true;
					maze_flag_pub.publish(maze_flag_msg);
					//car_control_speed_msg.data = 200;


					sonar_flag_msg.data = 1;
					sonar_flag_pub.publish(sonar_flag_msg);
					
					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					 if(stop_sign_flag == true)
					{
						ROS_INFO("Mission First Stop Detection");
						
						car_control_speed_msg.data = 0;
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						ros::Duration(3.0).sleep();
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 10;

					}
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

			case 10: //장애물 회피 살짝 직진
					sonar_flag_msg.data = 0;
					sonar_flag_pub.publish(sonar_flag_msg);
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					car_control_speed_topic_pub.publish(car_control_speed_msg);				
					target_yaw_msg.data = car_yaw_angle_d_temp;
					target_yaw_pub.publish(target_yaw_msg);
					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(car_move_distance > 0.5){
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 11;
					}
					break;

			case 11: //장애물 회피 우회전
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					car_control_speed_topic_pub.publish(car_control_speed_msg);				
					target_yaw_msg.data = car_yaw_angle_d_temp + 65;
					target_yaw_pub.publish(target_yaw_msg);
					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(car_move_distance > 2.2){
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 12;
					}
					break;

			case 12: //장애물 회피 좌회전
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					car_control_speed_topic_pub.publish(car_control_speed_msg);				
					target_yaw_msg.data = car_yaw_angle_d_temp -55;
					target_yaw_pub.publish(target_yaw_msg);
					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(car_move_distance > 2.2){
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 13;
					}
					break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

			case 13: //오른쪽 차선 유지
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);

					overtake_flag_msg.data = true;
					overtake_flag_pub.publish(overtake_flag_msg);

					overtake_lane_msg.data = 0.85 + (0.42*2);
					overtake_lane_control_pub.publish(overtake_lane_msg);

					yaw_control_mode_msg.data = OVERTAKE_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;				

					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(car_linear_maze <= 0.8){ // 장애물 검출
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 14;
					}
					break;

			case 14: //장애물 회피 좌회전
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					car_control_speed_topic_pub.publish(car_control_speed_msg);				
					target_yaw_msg.data = car_yaw_angle_d_temp -65;
					target_yaw_pub.publish(target_yaw_msg);
					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(car_move_distance > 1.9){
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 15;
					}
					break;

			case 15: //장애물 회피 우회전
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);
					yaw_control_mode_msg.data = YAW_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;
					car_control_speed_topic_pub.publish(car_control_speed_msg);				
					target_yaw_msg.data = car_yaw_angle_d_temp + 65;
					target_yaw_pub.publish(target_yaw_msg);
					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(car_move_distance > 1.9){
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 16;
					}
					break;

			case 16: //중앙 차선 유지
					lane_control_flag_msg.data = false;
					lane_control_topic_pub.publish(lane_control_flag_msg);

					overtake_flag_msg.data = true;
					overtake_flag_pub.publish(overtake_flag_msg);

					overtake_lane_msg.data = 0.85 + 0.42;
					overtake_lane_control_pub.publish(overtake_lane_msg);

					yaw_control_mode_msg.data = OVERTAKE_CONTROL;
					yaw_control_mode_pub.publish(yaw_control_mode_msg);

					car_move_distance =  car_linear_odom - car_linear_odom_temp;				

					car_control_speed_msg.data = 200;
					car_control_speed_topic_pub.publish(car_control_speed_msg);

					if(stop_sign_flag == true){ // 정지선 검출
						car_linear_odom_temp = car_linear_odom;
						car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
						mission_flag = 17;
					}
					break;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

					
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	

			case 30: //콘 트래킹 후 라인트레이싱 (T자 주차 직전)
				lane_control_flag_msg.data = true;
				lane_control_topic_pub.publish(lane_control_flag_msg);

				yaw_control_mode_msg.data = LANE_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = 200;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if (stop_sign_flag == true)
				{
					ROS_INFO("Mission parking Stop Detection");
					car_control_speed_msg.data = 0;
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					ros::Duration(2.0).sleep();

					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					car_linear_odom_temp = car_linear_odom;								    // odom 초기화

					mission_flag = 31;
				}
				break;

			case 31:  //살짝 직진
				lane_control_flag_msg.data = false;
				lane_control_topic_pub.publish(lane_control_flag_msg);

				car_move_distance = car_linear_odom - car_linear_odom_temp;

				target_yaw = car_yaw_angle_d_temp;
				target_yaw_msg.data = target_yaw;
				target_yaw_pub.publish(target_yaw_msg);

				yaw_control_mode_msg.data = YAW_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = 200;
				car_control_speed_topic_pub.publish(car_control_speed_msg);
				printf("distance: %6.3lf\n", car_move_distance);

				if ((car_move_distance >= 2.6))										// 처음 회전각을 위해 전진 하는거 수치 수정 요망
				{
					car_control_speed_msg.data = 180;
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					car_linear_odom_temp = car_linear_odom;
					mission_flag = 32;
				}
				break;

			case 32: //90도 이상 꺽어서 들어가는거
				lane_control_flag_msg.data = false;
				lane_control_topic_pub.publish(lane_control_flag_msg);


				target_yaw = car_yaw_angle_d_temp - 120;
				target_yaw_msg.data = target_yaw;
				target_yaw_pub.publish(target_yaw_msg);

				yaw_control_mode_msg.data = YAW_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);


				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				car_move_distance = car_linear_odom - car_linear_odom_temp;

				if ((car_move_distance > 3.0))										// 처음 회전각을 위해 전진 하는거 수치 수정 요망
				{
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					car_linear_odom_temp = car_linear_odom;
					mission_flag = 33;
				}
				break;
			
			case 33: //라인 트레이싱으로 자세 제어
				lane_control_flag_msg.data = true;
				lane_control_topic_pub.publish(lane_control_flag_msg);

				yaw_control_mode_msg.data = LANE_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				car_move_distance = car_linear_odom - car_linear_odom_temp;

				if ((car_move_distance > 6.5))												// 처음 회전각을 위해 전진 하는거 수치 수정 요망
				{
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					car_linear_odom_temp = car_linear_odom;
					mission_flag = 34;
				}
				break;

			case 34: //imu를 이용하여 yaw 제어로 정지선까지 직진
				lane_control_flag_msg.data = false;
				lane_control_topic_pub.publish(lane_control_flag_msg);

				target_yaw = car_yaw_angle_d_temp;
				target_yaw_msg.data = target_yaw + 3;
				target_yaw_pub.publish(target_yaw_msg);

				yaw_control_mode_msg.data = YAW_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_move_distance = car_linear_odom - car_linear_odom_temp;

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if ((car_move_distance > 1.82))
				{
					car_control_speed_msg.data = 0;
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					ros::Duration(2.0).sleep();

					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					car_linear_odom_temp = car_linear_odom;

					mission_flag = 35;
				}
				break;

			case 35: //후진(직선)
				yaw_control_mode_msg.data = 6;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = -180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				target_yaw = car_yaw_angle_d_temp + 0;
				target_yaw_msg.data = target_yaw;
				target_yaw_pub.publish(target_yaw_msg);

				car_move_distance = car_linear_odom_temp - car_linear_odom;

				if ((car_move_distance > 0.70))
				{
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 36;
				}
				break;

			case 36: //후진 (T자 주차)
				yaw_control_mode_msg.data = 6;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = -180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				target_yaw = car_yaw_angle_d_temp + 180;
				target_yaw_msg.data = target_yaw;
				target_yaw_pub.publish(target_yaw_msg);

				car_move_distance = car_linear_odom_temp - car_linear_odom;

				if (car_move_distance > 2.6)
				{
					car_linear_odom_temp = car_linear_odom;
					//car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 37;
				}
				break;
			
			case 37: //후진 (T자 주차)
				yaw_control_mode_msg.data = 6;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = -180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				target_yaw = car_yaw_angle_d_temp + 95;
				target_yaw_msg.data = target_yaw;
				target_yaw_pub.publish(target_yaw_msg);

				car_move_distance = car_linear_odom_temp - car_linear_odom;

				if (car_move_distance > 1.1)
				{
					car_linear_odom_temp = car_linear_odom;
					//car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 38;
				}
				break;

			case 38: //주차 후 정지
				car_control_speed_msg.data = 0;
				car_control_speed_topic_pub.publish(car_control_speed_msg);
				ros::Duration(2.0).sleep();

				mission_flag = 39;

				car_linear_odom_temp = car_linear_odom;
				car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;
				break;
				
			case 39: //주차 후 나가기 직진
				yaw_control_mode_msg.data = YAW_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);
				car_move_distance = car_linear_odom - car_linear_odom_temp;

				target_yaw = car_yaw_angle_d_temp;
				target_yaw_msg.data = target_yaw;
				target_yaw_pub.publish(target_yaw_msg);

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if (car_move_distance >= 0.06)
				{
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 40;
				}
				break;

			case 40: //주차 후 나가기 턴
				yaw_control_mode_msg.data = YAW_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);
				car_move_distance = car_linear_odom - car_linear_odom_temp;

				target_yaw = car_yaw_angle_d_temp;
				target_yaw_msg.data = target_yaw + 170;
				target_yaw_pub.publish(target_yaw_msg);

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if (car_move_distance >= 2.0)
				{
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 41;
				}
				break;

			case 41: //주차 후 나가기 라인 추종으로 나가기
				lane_control_flag_msg.data = true;
				lane_control_topic_pub.publish(lane_control_flag_msg);
				car_move_distance = car_linear_odom - car_linear_odom_temp;

				yaw_control_mode_msg.data = LANE_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if (car_move_distance >= 4.0)
				{
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 42;
				}
				break;

			case 42: //미로 주행을 위해 턴
				yaw_control_mode_msg.data = YAW_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);
				car_move_distance = car_linear_odom - car_linear_odom_temp;

				target_yaw = car_yaw_angle_d_temp;
				target_yaw_msg.data = target_yaw - 90;
				target_yaw_pub.publish(target_yaw_msg);

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if (car_move_distance >= 1.5)
				{
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = (imu1_heading_angle_radian * 180) / M_PI;			// heading 초기화
					mission_flag = 43;
				}
				break;

			case 43: //정지선까지 라인 추종(주차 끝)
				lane_control_flag_msg.data = true;
				lane_control_topic_pub.publish(lane_control_flag_msg);
				car_move_distance = car_linear_odom - car_linear_odom_temp;

				yaw_control_mode_msg.data = LANE_CONTROL;
				yaw_control_mode_pub.publish(yaw_control_mode_msg);

				car_control_speed_msg.data = 180;
				car_control_speed_topic_pub.publish(car_control_speed_msg);

				if (stop_sign_flag == true)
				{
					ROS_INFO("Mission First Stop Detection");

					car_control_speed_msg.data = 0;
					car_control_speed_topic_pub.publish(car_control_speed_msg);
					ros::Duration(3.0).sleep();
					car_linear_odom_temp = car_linear_odom;
					car_yaw_angle_d_temp = RAD2DEG(imu1_heading_angle_radian);
					mission_flag = 44;

				}
				break;
				//////////////////////////////미로 시작////////////////////////////////////////////
				case 44:
					{
						sonar_flag_msg.data = 0;
						sonar_flag_pub.publish(sonar_flag_msg);
						car_linear_odom_temp = car_linear_odom;								    // odom 초기화
						mission_flag ++;
					}
					break;
				case 45:
					{
						car_control_speed_msg.data = 120;
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						car_move_distance =  car_linear_odom - car_linear_odom_temp;
						sonar_flag_msg.data = 0;
						sonar_flag_pub.publish(sonar_flag_msg);
						if(car_move_distance > 1.0)
						{
							mission_flag ++;
						}
					}
					break;
							
					
				case 46:
						lane_control_flag_msg.data = false;
						lane_control_topic_pub.publish(lane_control_flag_msg);
						maze_control_flag_msg.data = true;
						maze_control_topic_pub.publish(maze_control_flag_msg);
						yaw_control_mode_msg.data = 4;
						yaw_control_mode_pub.publish(yaw_control_mode_msg);
						car_control_speed_msg.data = 120;                                       // stop
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						printf("distance : %6.3lf\n",car_linear_maze);
						sonar_flag_msg.data = 0;
						sonar_flag_pub.publish(sonar_flag_msg);
						
						if(car_linear_maze<=(float)obs_distance)
						{
							car_control_speed_msg.data = 0;                                       // stop
							car_control_speed_topic_pub.publish(car_control_speed_msg);
							//ros::Duration(0.5).sleep();
							car_linear_odom_temp = car_linear_odom;
							mission_flag += 2;
						}
						break;
				case 47:
						car_control_speed_msg.data = 0;                                       // stop
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						yaw_control_mode_msg.data = 4;
						mission_flag += 1;
						printf("distance : %6.3lf\n",car_linear_maze);
						//ros::Duration(3.0).sleep();
						break;
				
				case 48:
						yaw_control_mode_msg.data = 5;
						yaw_control_mode_pub.publish(yaw_control_mode_msg);
						
						car_move_distance =  car_linear_odom - car_linear_odom_temp;
						
						car_control_speed_msg.data = 160;                                       // stop
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						
						sonar_flag_msg.data = 0;
						sonar_flag_pub.publish(sonar_flag_msg);
						//car_yaw_angle_d_temp = imu1_heading_angle_radian;
						//target_yaw = car_yaw_angle_d_temp  - 44;
						//target_yaw_msg.data = target_yaw;
						//target_yaw_pub.publish(target_yaw_msg);
						
						printf("distance : %6.3lf\n",car_move_distance);
						
						if(car_move_distance >= 6.2)
						{
							//car_control_speed_msg.data = -180;                                       // stop
							//car_control_speed_topic_pub.publish(car_control_speed_msg);
							car_linear_odom_temp = car_linear_odom;
							mission_flag ++;
						}
					
						break;
				case 49:
						lane_control_flag_msg.data = false;
						lane_control_topic_pub.publish(lane_control_flag_msg);
						
						yaw_control_mode_msg.data = 4;
						yaw_control_mode_pub.publish(yaw_control_mode_msg);
						car_control_speed_msg.data = 160;                                       // stop
						car_control_speed_topic_pub.publish(car_control_speed_msg);
						printf("distance : %6.3lf\n",car_linear_maze);
						if(car_move_distance >= 4.0)
						{
							mission_flag ++;
						}
						break;
							
				case 50:
						lane_control_flag_msg.data = true;
						lane_control_topic_pub.publish(lane_control_flag_msg);
						maze_control_flag_msg.data = false;
						maze_control_topic_pub.publish(maze_control_flag_msg);
						yaw_control_mode_msg.data = LANE_CONTROL;
						yaw_control_mode_pub.publish(yaw_control_mode_msg);
						break;
		}
	
		headangle_msg.data = imu1_heading_angle_radian;
		imu_heading_angle_radian_pub.publish(headangle_msg);
		loop_rate.sleep();
		ros::spinOnce();

		++count;
	}
	return 0;
}







