#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;

cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_detection_color");
    ros::NodeHandle nh;
    ros::Rate loopRate(15);

	ROS_INFO("****** Starting Object Detection (COLOR) Node ... ... ******");

	image_transport::ImageTransport colorImageTransport(nh);
	image_transport::ImageTransport depthImageTransport(nh);
	image_transport::ImageTransport processImageTransport(nh);
	image_transport::ImageTransport monitorImageTransport(nh);
	image_transport::Publisher colorImagePub=colorImageTransport.advertise("/vision/d435i/color",1);
	image_transport::Publisher depthImagePub=depthImageTransport.advertise("/vision/d435i/depth",1);
	image_transport::Publisher processImagePub=processImageTransport.advertise("/vision/process",1);
	image_transport::Publisher monitorImagePub=monitorImageTransport.advertise("/vision/monitor",1);
	ros::Publisher targetPosePub=nh.advertise<geometry_msgs::PoseStamped>("/vision/target/pose",1);

	//load calibrate data
	const bool isEyeInHand = false;
	cv::Matx44d camera2Base;
	cv::Matx44d camera2End;

	//load calibration file path
	string calibration_info_url;
	if(!isEyeInHand){
		ros::param::get("object_detection_color/base2Camera_calibration_url",calibration_info_url);
	}else{
		ros::param::get("object_detection_color/camera2End_calibration_url",calibration_info_url);
	}

	if (!isEyeInHand) {
		cv::FileStorage base2CameraCalibrationFile(calibration_info_url, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
		cv::Mat base2CameraRotate;
		cv::Mat base2CameraTranslation;
		base2CameraCalibrationFile["rotate"] >> base2CameraRotate;
		base2CameraCalibrationFile["translation"] >> base2CameraTranslation;
		base2CameraCalibrationFile.release();

		cv::Mat camera2BaseRotate = base2CameraRotate.inv(cv::DECOMP_SVD);
		cv::Mat camera2BaseTranslation = -base2CameraRotate.inv(cv::DECOMP_SVD)*base2CameraTranslation;
		camera2Base = {
			camera2BaseRotate.at<double>(0,0),	camera2BaseRotate.at<double>(0,1),	camera2BaseRotate.at<double>(0,2),	camera2BaseTranslation.at<double>(0,0),
			camera2BaseRotate.at<double>(1,0),	camera2BaseRotate.at<double>(1,1),	camera2BaseRotate.at<double>(1,2),	camera2BaseTranslation.at<double>(1,0),
			camera2BaseRotate.at<double>(2,0),	camera2BaseRotate.at<double>(2,1),	camera2BaseRotate.at<double>(2,2),	camera2BaseTranslation.at<double>(2,0),
			0,									0,									0,									1 };
	} else {
		cv::FileStorage camera2EndCalibrationFile(calibration_info_url, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
		cv::Mat camera2EndRotate;
		cv::Mat camera2EndTranslation;
		camera2EndCalibrationFile["rotate"] >> camera2EndRotate;
		camera2EndCalibrationFile["translation"] >> camera2EndTranslation;
		camera2EndCalibrationFile.release();

		camera2End = {
			camera2EndRotate.at<double>(0,0),	camera2EndRotate.at<double>(0,1),	camera2EndRotate.at<double>(0,2),	camera2EndTranslation.at<double>(0,0),
			camera2EndRotate.at<double>(1,0),	camera2EndRotate.at<double>(1,1),	camera2EndRotate.at<double>(1,2),	camera2EndTranslation.at<double>(1,0),
			camera2EndRotate.at<double>(2,0),	camera2EndRotate.at<double>(2,1),	camera2EndRotate.at<double>(2,2),	camera2EndTranslation.at<double>(2,0),
			0,									0,									0,									1 };
	}

	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_BGR8);
	cfg.enable_stream(RS2_STREAM_DEPTH);
	pipe.start(cfg);

	rs2::pipeline_profile profile = pipe.get_active_profile();
	auto dev = profile.get_device();
	auto sensor = profile.get_device().first<rs2::depth_sensor>();
	sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);

	rs2::video_stream_profile colorstreamprofile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	rs2::stream_profile depthstreamprofile = profile.get_stream(RS2_STREAM_DEPTH);

	rs2_intrinsics intr = colorstreamprofile.get_intrinsics();
	cv::Matx33f cameraMatrix = {	intr.fx,	0,			intr.ppx,
									0,			intr.fy,	intr.ppy,
									0,			0,			1 };
	vector<float> distCoeffs(begin(intr.coeffs), end(intr.coeffs));

	rs2::align align2Color(RS2_STREAM_COLOR);
	rs2::colorizer colorizerDepth;

	while (ros::ok()) {
		auto frames = pipe.wait_for_frames();
		frames = frames.apply_filter(align2Color).apply_filter(colorizerDepth);

		auto colorFrame = frames.get_color_frame();
		auto depthFrame = frames.get_depth_frame();
		auto depthColorFrame = frames.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8).as<rs2::video_frame>();

		cv::Mat colorImage(colorFrame.get_height(),colorFrame.get_width(),CV_8UC3,const_cast<void *>(colorFrame.get_data()));
		cv::Mat depthColorImage(depthColorFrame.get_height(), depthColorFrame.get_width(), CV_8UC3, const_cast<void *>(depthColorFrame.get_data()));
		cvtColor(depthColorImage, depthColorImage, CV_RGB2BGR);

		sensor_msgs::ImagePtr colorImagePtr=cv_bridge::CvImage(std_msgs::Header(),"bgr8",colorImage).toImageMsg();
		sensor_msgs::ImagePtr depthImagePtr=cv_bridge::CvImage(std_msgs::Header(),"bgr8",depthColorImage).toImageMsg();
		colorImagePub.publish(colorImagePtr);
		depthImagePub.publish(depthImagePtr);

		cv::Mat processImage = colorImage.clone();
		for (cv::MatIterator_<cv::Vec3b> it = processImage.begin<cv::Vec3b>(); it != processImage.end<cv::Vec3b>(); ++it) {
			if (2 * (*it)[0] > (*it)[2] || 2 * (*it)[1] > (*it)[2]) {
				*it = { 0,0,0 };
			} 
		}
		erode(processImage, processImage, cv::Mat(), cv::Point(-1, -1), 3);
		cvtColor(processImage, processImage, CV_BGR2GRAY);
		
		sensor_msgs::ImagePtr processImagePtr=cv_bridge::CvImage(std_msgs::Header(),"mono8",processImage).toImageMsg();
		processImagePub.publish(processImagePtr);

		vector<cv::Vec3f> circles;
		HoughCircles(processImage, circles, cv::HOUGH_GRADIENT, 1.5, 100, 300, 20, 30, 70);
		for (auto &&circleInfo : circles) {
			circle(colorImage, cv::Point(circleInfo[0], circleInfo[1]), circleInfo[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
		}

		cv::Mat monitorImage;
		cv::addWeighted(colorImage, 0.8, depthColorImage, 0.2, 0, monitorImage);
		sensor_msgs::ImagePtr monitorImagePtr=cv_bridge::CvImage(std_msgs::Header(),"bgr8",monitorImage).toImageMsg();
		monitorImagePub.publish(monitorImagePtr);

		if (circles.size() > 0)
		{
			float target2CameraTranslation[3] = {0};
			float pixel[2] = {circles[0][0], circles[0][1]};
			rs2_deproject_pixel_to_point(target2CameraTranslation, &intr, pixel, depthFrame.get_distance(circles[0][0], circles[0][1]));
			cv::Matx44d target2Camera = {1, 0, 0, target2CameraTranslation[0],
										 0, 1, 0, target2CameraTranslation[1],
										 0, 0, 1, target2CameraTranslation[2],
										 0, 0, 0, 1};

			if (!isEyeInHand)
			{
				cv::Matx44d target2Base = camera2Base * target2Camera;
				//Mat target2BaseRotate = Mat::eye(3, 3, CV_64FC1) * target2Base.get_minor<3, 3>(0, 0);
				cout << "target2Base: " << target2Base << endl;

				geometry_msgs::PoseStamped targetPose;
				targetPose.header = std_msgs::Header();
				targetPose.pose.position.x = target2Base(0, 3);
				targetPose.pose.position.y = target2Base(1, 3);
				targetPose.pose.position.z = target2Base(2, 3);
				targetPose.pose.orientation.x = 0;
				targetPose.pose.orientation.y = 0;
				targetPose.pose.orientation.z = 0;
				targetPose.pose.orientation.w = 1;

				targetPosePub.publish(targetPose);
			}
			else
			{
				//todo
			}
		}

		loopRate.sleep();

		//Mat edge;
		//Canny(colorImage, edge, 50, 100, 3, true);
		//morphologyEx(colorImage, colorImage, MORPH_CLOSE,Mat(),Point(-1,-1),3);
		//cvtColor(colorImage, colorImage, CV_BGR2GRAY);
		//vector<vector<Point>> t;
		//findContours(colorImage, t, cv::noArray(), RETR_LIST, CHAIN_APPROX_SIMPLE);
		//double maxArea = 0;
		//vector<Point> *tc;
		//for (auto &curve : t) {
		//	double curveArea = contourArea(curve);
		//	if(curveArea>maxArea){
		//		maxArea = curveArea;
		//		tc = &curve;
		//	}
		//}
		//rectangle(colorImage, boundingRect(*tc), Scalar(100, 0, 0));
	}

	ROS_INFO("****** Stopped Object Detection (COLOR) Node ! ******");

	return 0;

}

