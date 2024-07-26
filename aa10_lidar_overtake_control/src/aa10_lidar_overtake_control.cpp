#define DEBUG 1
#define DEBUG_ROS_INFO 1 

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include "nav_msgs/Odometry.h"


#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"


#include "geometry_msgs/Pose2D.h"

#include "laser_geometry/laser_geometry.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "obstacle_blob_processing.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/convex_hull.h>

// Image processing blob analysis
#include <string.h>
#include <iostream>
#include <sstream>
#include <cmath>     

#include <opencv2/opencv.hpp>

// unit : m

#define OFF 0
#define ON  1

#define IMG_HEIGHT_LIDAR  300
#define IMG_WIDTH_LIDAR   800
#define MAX_CON_NO  200

#define LANE_LEFT    -1
#define LANE_CENTER   0
#define LANE_RIGHT    1

#define LANE_WIDTH   0.4
#define DETECT_RANGE 2.0 

#define IMAGE_SCALE  200

//using namespace cv;
using namespace std;
using namespace cv;

typedef struct
{
  float x;  // x
  float y;  // y
  float z;  // z
  float i;  // intensity
  uint16_t r;  // ring
} Point_3D_lidar;


double car_target_angle                = 0.0;
int    number_of_obstacle_point        = 10;
double obstace_avoidance_min_distance  = 1.5;
double lidar_offset_y                  = 0.0;

double Robot_Width                  = 0.24;
double Robot_Width_Tolerance        = 0.1; 

double roll,pitch,yaw;
double roll_d,pitch_d,yaw_d;

double obstacle_avoid_heading_angle = 0;

int heading_angle = 0;

int min_blob_area              = 200;
int max_blob_area              = 500;   
int sensitivity				   = 0; 

double lidar3d_roi_height_max  =  0.6;
double lidar3d_roi_height_min  = -0.2;
double lidar3d_roi_width_x     =  2.0;  

double obstalce_distance_x     = -1.0;

bool overtake_mode_flag        = false;
double wall_angle              = 0.0;
double overtake_xte            = 0.0;
int lane_pos                   = LANE_CENTER;
double overtake_ahead_x        = 0.0;
double overtake_ahead_y        = 0.4;

double target_overtake_lane_x  = 1.25;   

//////////////////////   ROS  //////////////////////////////
ros::Publisher overtake_lane_xte_pub;

/////////////////////// opnecv /////////////////////////////
cv::Mat Image_Obstacle;
cv::Mat Image_Obstacle_color;
cv::Mat Image_dilate;

cv::Mat mat_image_org_color;
cv::Mat mat_image_roi_crop; 



cv::Mat Region_of_Interest_crop(cv::Mat image, cv::Point* points)
{
	cv::Mat img_roi_crop_temp;
	cv::Mat image2;
	image.copyTo(image2);

	Rect bounds(0, 0, image2.cols, image2.rows);
	Rect r(points[0].x, points[0].y, points[2].x-points[0].x, points[2].y - points[0].y);
	img_roi_crop_temp = image2(r & bounds);

	return img_roi_crop_temp;
}


void heading_angle_Callback(const std_msgs::Float32& msg) 
{
	yaw_d = msg.data;

	// printf("%6.3lf(rad)  %6.3lf \n",yaw, yaw*180/3.14159);		  
}
 
double detect_car_lane(cv::Mat lidar_image)
{
	double lane = 0;
	
	
	return lane;
} 

double wall_distance_calculation(cv::Mat lidar_image)
{
	int max_left_line_distance_i   = -1;
	double wall_angle              = 0.0;
	double max_line_length         = 0.0;
	double wall_distance           = 0.0;
	double c,d;
	double overtake_ahead_image_x  = 0.0;
	double overtake_ahead_image_y  = 0.0;
	std_msgs::Float32 overtake_ahead_x_msg;
		
	vector<Vec4i> linesP;
	HoughLinesP(lidar_image, linesP, 1, CV_PI / 180, 10, 10, 30);
	
	printf("Line Number : %3d\n", (int)linesP.size());
	
	double x1, x2;
	double y1, y2;
	for(int i = 0; i < linesP.size(); i++)
	{
		Vec4i L = linesP[i];
		double dis_x    = L[0] - L[2];
		double dis_y    = L[1] - L[3];		
		double distance = sqrt(dis_x * dis_x + dis_y * dis_y);
		
		if( (max_line_length  <= distance) && (L[0] <=IMG_WIDTH_LIDAR/2) && (L[2] <=IMG_WIDTH_LIDAR/2) )     // 추가해야 함 왼쪽 라인만 검
		{
			max_line_length          = distance;
			max_left_line_distance_i = i;
			x1 = (double)L[0];  y1 = (double)L[1];
			x2 = (double)L[2];  y2 = (double)L[3];
		}
		
		line(Image_Obstacle_color, cv::Point(L[0],L[1]), cv::Point(L[2],L[3]),  Scalar(255,100, 0), 2, LINE_AA);
	}
	
	//printf("%2d max_line_distance %5.1lf\n", max_line_distance_i, max_line_length);
	
	line(Image_Obstacle_color, cv::Point(x1,y1), cv::Point(x2,y2),  Scalar(0,100,255), 4, LINE_AA);
	line(Image_Obstacle_color, cv::Point(x1,y1), cv::Point(x2,y2),  Scalar(0,100,255), 4, LINE_AA);
	
	c = -(x2 - x1)/ ( y2 - y1 );
	d = c * y1 + x1;
	 
	wall_angle             = RAD2DEG( atan2( (x2 - x1), ( y2 - y1 ) ) );
	wall_distance          = (IMG_WIDTH_LIDAR/2 - d)/IMAGE_SCALE;
	
	overtake_ahead_image_y = overtake_ahead_y * IMAGE_SCALE; 
	overtake_ahead_image_x = overtake_ahead_image_y * c + d; 
	overtake_ahead_x       = overtake_ahead_image_x / IMAGE_SCALE;
	printf("wall_angle : %5.2lf  wall_distance %5.2lf\nx = %5.2lf x y + %5.2lf  \nahead x = %5.2lf |  %5.2lf[m] \n", wall_angle, wall_distance, c, d, overtake_ahead_image_x, overtake_ahead_x);
	//printf("x = %5.2lf x y + %5.2lf  \n", c, d);
	
	//printf("ahead x = %5.2lf |  %5.2lf[m] \n", overtake_ahead_image_x, overtake_ahead_x  );
	
	overtake_ahead_x_msg.data = target_overtake_lane_x - overtake_ahead_x;
	overtake_lane_xte_pub.publish(overtake_ahead_x_msg);
		
	return wall_distance;
} 
 
 
double detect_car(cv::Mat lidar_image)
{
	cv::Mat front_obstacle_img;
	
	int obstacle_forward_check   = 0;
	int left_corner              = IMG_WIDTH_LIDAR/2 - (Robot_Width + Robot_Width_Tolerance)*IMAGE_SCALE;
	int right_corner             = IMG_WIDTH_LIDAR/2 + (Robot_Width + Robot_Width_Tolerance)*IMAGE_SCALE;
	
	int obstacle_limit_count     = 20;
	int obstalce_distance_i      = -1;
	
			
	double center_x      = IMG_WIDTH_LIDAR/2;
	double lane_width    = LANE_WIDTH/2 * IMAGE_SCALE/2;
	double detect_range  = DETECT_RANGE * IMAGE_SCALE;  
	
	unsigned int vertical_projection[IMG_HEIGHT_LIDAR];
	unsigned int horizontal_projection[IMG_WIDTH_LIDAR];
	
	memset(vertical_projection,    0, IMG_HEIGHT_LIDAR*sizeof(unsigned int) );
	memset(horizontal_projection,  0, IMG_WIDTH_LIDAR*sizeof(unsigned int) );
	
	
	Point points[4];
	
	points[0] = Point(center_x - lane_width, IMG_HEIGHT_LIDAR - detect_range);
	points[1] = Point(center_x - lane_width, IMG_HEIGHT_LIDAR);
	points[2] = Point(center_x + lane_width, IMG_HEIGHT_LIDAR);
	points[3] = Point(center_x + lane_width, IMG_HEIGHT_LIDAR - detect_range);
	
	//printf("%5.1lf %5.1lf %5.1lf \n", center_x - lane_width, center_x + lane_width, IMG_HEIGHT_LIDAR - detect_range);
	//front_obstacle_img = Region_of_Interest_crop(lidar_image,points);
	
	line(Image_Obstacle_color, cv::Point(IMG_WIDTH_LIDAR/2, 0),  cv::Point(IMG_WIDTH_LIDAR/2, IMG_HEIGHT_LIDAR), cv::Scalar(147,  20, 255), 3, 8, 0);  
	rectangle(Image_Obstacle_color, cv::Rect( cv::Point(center_x - lane_width, IMG_HEIGHT_LIDAR - detect_range), cv::Point(center_x + lane_width, IMG_HEIGHT_LIDAR) ),cv::Scalar(255, 255, 0), 2, 8, 0);  
	
	
	for(int i=0; i < IMG_HEIGHT_LIDAR ; i++)
	{
		for(int j=left_corner; j < right_corner ; j++)
		{
			vertical_projection[i] += ( (lidar_image.at<uchar>(i,j) == 255) ? 1 : 0);
		}
			//printf("%d %d\n",i,vertical_projection[i]);
	}
	
	for(int i =  IMG_HEIGHT_LIDAR - 1; i >= 0 ; i--)
	{
	
		if( vertical_projection[i] > obstacle_limit_count)
		{
			obstalce_distance_i = i;
			break;
		}		
	}
	
	obstalce_distance_x = (double)(IMG_HEIGHT_LIDAR - obstalce_distance_i )/IMAGE_SCALE;
	//printf("obstalce_distance_x : %3d - %2d =  %5.1lf[m]  \n\n",IMG_HEIGHT_LIDAR, obstalce_distance_i, obstalce_distance_x);
	
	return obstalce_distance_x;
}


int currnet_lane_check(void)
{
	int lane_no = 0;
	
	
	return lane_no;
}     

void lidar2d_pointcloud2_Callback(const sensor_msgs::PointCloud2ConstPtr& scan)
{
	unsigned int num_points = scan->width;
	double front_obstacle_distance = 0;
	//printf("The number of points in the input pointcloud is %i\n", num_points);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*scan, *cloud);	
		
	pcl::PCLPointCloud2 cloud_p;
	pcl::toPCLPointCloud2(*cloud, cloud_p);
			
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_p, output);
	output.header.frame_id = "lidar_scan";
		
	pcl::PointCloud<pcl::PointXYZ> _2d_cloud;
	pcl::fromROSMsg(output,_2d_cloud);

	//printf("%d\n",laserCouldIn.points.size());
	Image_Obstacle        = cv::Mat::zeros(IMG_HEIGHT_LIDAR, IMG_WIDTH_LIDAR, CV_8UC1); 
	Image_Obstacle_color  = cv::Mat::zeros(IMG_HEIGHT_LIDAR, IMG_WIDTH_LIDAR, CV_8UC3);
	
	for(unsigned int j =0; j <_2d_cloud.points.size();j++)
	{
		int img_x,img_y;
	
		if( (_2d_cloud.points[j].x != 0.0) && (_2d_cloud[j].y != 0.0) )
		{
			img_x = IMG_WIDTH_LIDAR/2 + (int)(_2d_cloud.points[j].y * IMAGE_SCALE);
			img_y = IMG_HEIGHT_LIDAR  - (int)(_2d_cloud.points[j].x * IMAGE_SCALE);
			//printf(" %d %d \n",img_x,img_y);
			if( ( ( img_x>=0) && (img_x < IMG_WIDTH_LIDAR)   ) &&  ( (img_y >=0) && (img_y<IMG_HEIGHT_LIDAR) ) )  
			{
				Image_Obstacle.at<uchar>(img_y, img_x) = 255;
			}	
		} 
	}	       
		
		//////////////////////////////////////// 영상처리/////////////////////////////////////////
	
	dilate(Image_Obstacle, Image_dilate, cv::Mat::ones(cv::Size(5, 5), CV_8UC1), cv::Point(-1, -1), 1);
	
	front_obstacle_distance = detect_car(Image_dilate);
	printf("front_line_obstacle_distance_x : %5.1lf[m]\n\n",front_obstacle_distance );
	wall_distance_calculation(Image_Obstacle);		
	
	cv::imshow("Lidar window", Image_dilate);
	cv::imshow("result", Image_Obstacle_color);
} 

void overtake_control_mode_Callback(const std_msgs::Bool& msg) 
{
	overtake_mode_flag = msg.data;
}

void  target_overtake_lane_x_Callback(const std_msgs::Float32& msg)
{
	target_overtake_lane_x = msg.data;
}

int main(int argc, char **argv)
{
	char buf[2];
	ros::init(argc, argv, "aa10_lidar_overtake_control");

	int  img_threshold1 =  110;
	int  img_threshold2 =  150;
	
	int  area1          =  500;
	int  area2          = 1400;
	
	ros::NodeHandle n;  
	std::string lidar2d_cloud_topic                      = "/pointcloud2d";
	std::string imu_heading_angle_topic                  = "/robor/imu/yaw_degree";
	std::string front_obstacle_distance_topic            = "/lidar/front_obstacle_distance";
	
	std::string overtake_control_mode_topic              = "/flag/overtake_mode";
	std::string target_overtake_lane_x_topic             = "/target/overtake_lane_x"; 

	/*other*/
	ros::param::get("~lidar2d_cloud_topic",                   lidar2d_cloud_topic);
	ros::param::get("~imu_heading_angle_topic",               imu_heading_angle_topic); 
	ros::param::get("~front_obstacle_distance_topic",         front_obstacle_distance_topic);	
	
	ros::param::get("~overtake_control_mode_topic",           overtake_control_mode_topic);
	
	ros::param::get("~sensitivity",           				  sensitivity); 
	
	ros::Publisher  obstacle_detect_distance_pub         = n.advertise<std_msgs::Float32>(front_obstacle_distance_topic, 1);
	
	ros::Subscriber sub_heading__angle                   = n.subscribe(imu_heading_angle_topic, 1, heading_angle_Callback);
	ros::Subscriber sub_overtake_control_mode_flag       = n.subscribe(overtake_control_mode_topic, 1, overtake_control_mode_Callback);
	
	ros::Subscriber sub_target_overtake_lane_x           = n.subscribe(target_overtake_lane_x_topic, 1,target_overtake_lane_x_Callback);
	
	ros::Subscriber sub_2d_lidar                         = n.subscribe(lidar2d_cloud_topic, 10, lidar2d_pointcloud2_Callback);

	
	
	 
	
	
	overtake_lane_xte_pub                                = n.advertise<std_msgs::Float32>("/xte/overtake_lane",1);
	
	////////////////   imu _sensor //////////////////////
	sensor_msgs::Imu imu_data;
	roll = pitch = yaw = 0.0;

	/**
	* A count of how many messages we have sent. This is used to create
	* a unique string for each message.
	*/
	int count = 0;

	tf::TransformListener tf_listener;

	cv::namedWindow("Lidar window", cv::WINDOW_NORMAL);
	cv::resizeWindow("Lidar window", IMG_WIDTH_LIDAR/2,IMG_HEIGHT_LIDAR/2);
	cv::moveWindow("Lidar window", 100, 500);

	cv::namedWindow("result", cv::WINDOW_NORMAL);
	cv::resizeWindow("result", IMG_WIDTH_LIDAR/2,IMG_HEIGHT_LIDAR/2);

	ros::Rate loop_rate(30);  // 10
	
	std_msgs::Float32 obstalce_distance_x_msg;
	std_msgs::Float32 front_obstacle_distance_msg;
	//overtake_mode_flag = true;
	while (ros::ok())
	{
	/**
	* This is a message object. You stuff it with data, and then publish it.
	*/
		if(obstalce_distance_x != -1.0)
		{
			
			obstalce_distance_x_msg.data = obstalce_distance_x;
			obstacle_detect_distance_pub.publish(obstalce_distance_x_msg);			
		}
		
		if(overtake_mode_flag == true)
		{
			//wall_distance_calculation(Image_Obstacle);		
			printf("\n");
			
		}
			//////////
	
      
		if (cv::waitKey(25) >= 0)
		{
			
		}
		//printf("\n\n");
		loop_rate.sleep();
		ros::spinOnce();
		++count;
	}



	return 0;
}



