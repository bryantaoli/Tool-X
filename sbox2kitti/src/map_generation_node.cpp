#include "map_generation_node.h"
//This code is to read the .bag files in ROS and decode them into .png and .pcd
//Coordinate system transformation function is optional. 

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace cv;
using namespace std;

double trans_roll_  = 0.0;
double trans_pitch_ = 0.0;
double trans_yaw_   = 0.0;
double trans_tx_    = 0.0;
double trans_ty_    = 0.0;
double trans_tz_    = 0.0;
Eigen::Affine3f transform_matrix_ = Eigen::Affine3f::Identity();

pcl::PointCloud<pcl::PointXYZI> lidar_cloud;

long long lidar_index = 0;// todo 上限
bool camera_captured = 1;// sampling

bool init_camera_time = 0;
bool init_lidar_time = 0;

long long camera_base_time;
long long lidar_base_time;

double cameratime;
double lidartime;

//About Lidar Points Cloud (Read & Trans & Save)
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
	if(!init_lidar_time)
	{
		lidar_base_time = lidar->header.stamp.sec * 1e3 + lidar->header.stamp.nsec / 1e6;
		init_lidar_time = true;
	}

	long long lidar_delta_time = lidar->header.stamp.sec * 1e3 + lidar->header.stamp.nsec / 1e6 - lidar_base_time;
	ROS_INFO("get lidar : %lld ms", lidar_delta_time);
	lidartime = lidar->header.stamp.sec + lidar->header.stamp.nsec / 1e9;
	char s[200];
	sprintf(s, "/home/xspc/Documents/0Alldata/1028/Tools_RosBag2KITTI-master/catkin_ws/output/pcd/%06lld.pcd", lidar_index); 
	++lidar_index;
	
	string lidartimefilename("/home/xspc/Documents/0Alldata/1028/Tools_RosBag2KITTI-master/catkin_ws/output/lidartime_timestamp.txt");
	
	char l[200];
	sprintf(l, "%06lld.pcd", lidar_index); 
	ofstream lidartime_file;
	lidartime_file.open(lidartimefilename.c_str(), std::ios::app);
	lidartime_file << setprecision(19) << lidartime <<" "<< l << std::endl;
	
	//camera_captured = false;

	pcl::fromROSMsg(*lidar, lidar_cloud);

	//Building the transformation matrix
	transform_matrix_.translation() << trans_tx_, trans_ty_, trans_tz_;
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_yaw_ * M_PI / 180, Eigen::Vector3f::UnitZ()));
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_pitch_ * M_PI / 180, Eigen::Vector3f::UnitY()));
  	transform_matrix_.rotate(Eigen::AngleAxisf(trans_roll_ * M_PI / 180, Eigen::Vector3f::UnitX()));

  	//Transformation
	pcl::PointCloud<pcl::PointXYZI>::Ptr trans_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  	pcl::transformPointCloud(lidar_cloud, *trans_cloud_ptr, transform_matrix_);
	//Save in .pcd
	pcl::io::savePCDFileASCII (s, *trans_cloud_ptr);
}

//About Visual Image (Read & Save)
void cameraCallback(const sensor_msgs::Image::ConstPtr& camera)
{
	ROS_INFO("discard camerams");
	if(!init_camera_time)
	{
		camera_base_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6;
		init_camera_time = true;
	}

	long long camera_delta_time = camera->header.stamp.sec * 1e3 + camera->header.stamp.nsec / 1e6 - camera_base_time;
	if(camera_captured)
	{
		ROS_INFO("discard camera: %lld ms", camera_delta_time);
	}

	cameratime = camera->header.stamp.sec + camera->header.stamp.nsec / 1e9;
	ROS_INFO("get camera: %lld ms", camera_delta_time);
	char s[200];
	char undis[200];
	sprintf(s, "/home/xspc/Documents/0Alldata/1028/Tools_RosBag2KITTI-master/catkin_ws/output/png/%06lld.png", lidar_index-1); 
	string pngfilename("/home/xspc/Documents/0Alldata/1028/Tools_RosBag2KITTI-master/catkin_ws/output/png_timestamp.txt");
	ofstream png_file;

	char l[200];
	sprintf(l, "%06lld.pcd", lidar_index); 


	png_file.open(pngfilename.c_str(), std::ios::app);
	png_file << setprecision(19) << cameratime <<" "<< l << std::endl;
	sprintf(undis, "/home/xspc/Documents/0Alldata/1028/Tools_RosBag2KITTI-master/catkin_ws/output/undis/%06lld.png", lidar_index-1); 

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(camera, sensor_msgs::image_encodings::BGR8); 
		//cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", camera->encoding.c_str());
	}
	Mat img_rgb = cv_ptr->image;
	Mat img_rgb_undistor = cv_ptr->image;
	imwrite(s, img_rgb);

    // 去畸变的代码，目前编译有错
	// cv::Size img_sizea;
	// img_sizea.width=1280;
	// img_sizea.height=1024;
	// cv::Mat distortiona = img_rgb.clone();
	// cv::Mat camera_matrixa = (cv::Mat_<double>(3, 3) << 541.1788, 0.0, 644.1565, 0, 539.5916, 502.7171, 0, 0, 1);
	// cv::Mat distortion_coefficientsa=(cv::Mat_<double >(1,4)<<-0.07071596, -0.0031326, -0.00692476, 0.00191935);
	// cv::fisheye::undistortImage(img_rgb_undistor, distortiona, camera_matrixa, distortion_coefficientsa, camera_matrixa, img_sizea);
	// img_rgb_undistor = distortiona;
	// imwrite(undis, img_rgb_undistor);
	camera_captured = true;
}

int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "map_generation");

	ros::NodeHandle nh;
	ros::Subscriber sub_image = nh.subscribe("/image_raw", 1000, &cameraCallback);
	ros::Subscriber sub_lidar = nh.subscribe("/velodyne_points", 1000, &lidarCallback);
	ros::Rate loop_rate(50);
	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
