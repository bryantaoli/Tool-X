#include "pch.h"
#include <iostream>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <opencv2/core/eigen.hpp>
#include <iomanip>
#define fx 517.3
#define fy 516.5
#define cx 318.6
#define cy 255.3
using namespace std;
using namespace cv;
#define Ini_t_Thre 0.05
Point2d pixel2cam(const Point2d& p, const Mat& K)
{
	return Point2d
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
}

void pose_estimation_3d3d(
	const vector<Point3f>& pts1,
	const vector<Point3f>& pts2,
	Mat& R, Mat& t
);

void find_feature_matches(const Mat& img_1, const Mat& img_2, std::vector<KeyPoint>& keypoints_1, std::vector<KeyPoint>& keypoints_2, std::vector< DMatch >& matches)
{
	
	Ptr<FeatureDetector> detector = ORB::create();
	detector->detect(img_1, keypoints_1);
	detector->detect(img_2, keypoints_2);
	Mat descriptors_1, descriptors_2;
	Ptr<DescriptorExtractor> descriptor = ORB::create();
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
	descriptor->compute(img_1, keypoints_1, descriptors_1);
	descriptor->compute(img_2, keypoints_2, descriptors_2);
	vector<DMatch> onematch;
	matcher->match(descriptors_1, descriptors_2, onematch);
	double min_dist = 10000, max_dist = 0;
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = onematch[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (onematch[i].distance <= max(2 * min_dist, 30.0))
		{
			matches.push_back(onematch[i]);
		}
	}
}


int main()
{
	ifstream fin("./rgbd_dataset_freiburg2_xyz/rgb.txt");
	ifstream depthfile("./rgbd_dataset_freiburg2_xyz/depth.txt");
	ofstream out("./rgbd_dataset_freiburg2_xyz/outrgbd.txt");
	string line;
	getline(fin, line); 	out << line << endl;
	getline(fin, line); 	out << line << endl;
	getline(fin, line);	out << line << endl;

	string linedepth;
	getline(depthfile, linedepth);
	getline(depthfile, linedepth);
	getline(depthfile, linedepth);

	vector<string> rgb_files;
	vector<string> depth_files;
	vector<double> rgb_times;
	vector<double> depth_times;
	while (!fin.eof()){
		string rgb_time, rgb_file;
		fin >> rgb_time >> rgb_file;
		rgb_times.push_back(atof(rgb_time.c_str()));
		rgb_files.push_back("./rgbd_dataset_freiburg2_xyz/" + rgb_file);

		if (fin.good() == false)
			break;
	}
	while (!depthfile.eof()) {
		string depth_time, depth_file;
		depthfile >> depth_time >> depth_file;
		depth_times.push_back(atof(depth_time.c_str()));
		depth_files.push_back("./rgbd_dataset_freiburg2_xyz/" + depth_file);

		if (depthfile.good() == false)
			break;
	}

	//-- 读取图像
	Mat K = (Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
	int id_pic = 1;
	int id_depth = 1;

	while (1) {
		if (id_pic > rgb_files.size()-2)
		{
			return 0;
		}
		if (id_depth > depth_files.size()-2)
		{
			return 0;
		}
		Mat img_1 = imread(rgb_files[id_pic-1], CV_LOAD_IMAGE_COLOR);
		Mat img_2 = imread(rgb_files[id_pic], CV_LOAD_IMAGE_COLOR);
		vector<KeyPoint> keypoints_1, keypoints_2;
		vector<DMatch> matches;
		find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches);
		//cout << "一共找到了" << matches.size() << "组匹配点" << endl;

		if (fabs(depth_times[id_depth - 1] - rgb_times[id_pic - 1]) > fabs(depth_times[id_depth - 1] - rgb_times[id_pic]))
		{
			id_pic++;
		}
		// 建立3D点
		Mat depth1 = imread(depth_files[id_depth -1], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
		Mat depth2 = imread(depth_files[id_depth], CV_LOAD_IMAGE_UNCHANGED);       // 深度图为16位无符号数，单通道图像
		
		
		vector<Point3f> pts1, pts2;

		for (DMatch m : matches)
		{
			ushort d1 = depth1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
			ushort d2 = depth2.ptr<unsigned short>(int(keypoints_2[m.trainIdx].pt.y))[int(keypoints_2[m.trainIdx].pt.x)];
			if (d1 == 0 || d2 == 0)   // bad depth
				continue;
			Point2d p1 = pixel2cam(keypoints_1[m.queryIdx].pt, K);
			Point2d p2 = pixel2cam(keypoints_2[m.trainIdx].pt, K);
			float dd1 = float(d1) / 5000.0;
			float dd2 = float(d2) / 5000.0;
			pts1.push_back(Point3f(p1.x*dd1, p1.y*dd1, dd1));
			pts2.push_back(Point3f(p2.x*dd2, p2.y*dd2, dd2));
		}

		//cout << "3d-3d pairs: " << pts1.size() << endl;
		Mat R, t;
		pose_estimation_3d3d(pts1, pts2, R, t);
		//cout << "ICP via SVD results: " << endl;
		//cout << "R = " << R << endl;
		//cout << "t = " << t << endl;
		//cout << "R_inv = " << R.t() << endl;
		//cout << "t_inv = " << -R.t() *t << endl;
		Eigen::Matrix3d Reigen;
		Eigen::Vector3d teigen;
		cv2eigen(R, Reigen);
		cv2eigen(t, teigen);
		Eigen::Quaterniond q(Reigen);
		
		out  << setprecision(15) << rgb_times[id_pic] <<  ' ' << teigen(0)<<' '<< teigen(1)<<' '<< teigen(2)<<' ' << ' ' << q.x() << ' ' << q.y()<< ' '<<q.z()<<' ' << q.w()<<  endl;
		id_pic++; 
		id_depth++;
	}
	return 0;
}

void pose_estimation_3d3d(
	const vector<Point3f>& pts1,
	const vector<Point3f>& pts2,
	Mat& R, Mat& t
)
{
	Point3f p1, p2;     // center of mass
	int N = pts1.size();
	for (int i = 0; i < N; i++)
	{
		p1 += pts1[i];
		p2 += pts2[i];
	}
	p1 = Point3f(Vec3f(p1) / N);
	p2 = Point3f(Vec3f(p2) / N);
	vector<Point3f>     q1(N), q2(N); // remove the center
	for (int i = 0; i < N; i++)
	{
		q1[i] = pts1[i] - p1;
		q2[i] = pts2[i] - p2;
	}

	// compute q1*q2^T
	Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
	for (int i = 0; i < N; i++)
	{
		W += Eigen::Vector3d(q1[i].x, q1[i].y, q1[i].z) * Eigen::Vector3d(q2[i].x, q2[i].y, q2[i].z).transpose();
	}
	//cout << "W=" << W << endl;

	// SVD on W
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d U = svd.matrixU();
	Eigen::Matrix3d V = svd.matrixV();

	if (U.determinant() * V.determinant() < 0)
	{
		for (int x = 0; x < 3; ++x)
		{
			U(x, 2) *= -1;
		}
	}

	//cout << "U=" << U << endl;
	//cout << "V=" << V << endl;

	Eigen::Matrix3d R_ = U * (V.transpose());
	Eigen::Vector3d t_ = Eigen::Vector3d(p1.x, p1.y, p1.z) - R_ * Eigen::Vector3d(p2.x, p2.y, p2.z);

	// convert to cv::Mat
	R = (Mat_<double>(3, 3) <<
		R_(0, 0), R_(0, 1), R_(0, 2),
		R_(1, 0), R_(1, 1), R_(1, 2),
		R_(2, 0), R_(2, 1), R_(2, 2)
		);
	t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}
