#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "assist_tools.h"
#include "robot_jaka/RobotEndPosition.h"			// 仅用于眼在手上标定形式

using namespace std;

int main(int argc, char **argv)
{
	// 初始化ros节点，创建节点句柄，设置循环周期为15Hz
	ros::init(argc, argv, "object_detection_color");
    ros::NodeHandle nh;
    ros::Rate loopRate(15);
	ROS_INFO("****** Starting Object Detection (COLOR) Node ... ... ******");

	// 从ros参数服务器获取手眼标定形式（眼在手外或眼在手上），以及相应的标定文件存储路径
	// isEyeInHand == true：眼在手上		isEyeInHand == false：眼在手外	
	bool isEyeInHand;
	string calibrationInfoUrl;
	ros::param::get("camera_calibrate_hand_eye/isEyeInHand",isEyeInHand);
	if(isEyeInHand){
		ros::param::get("object_detection_color/eyeInHand_calibration_url",calibrationInfoUrl);
	}else{
		ros::param::get("object_detection_color/eyeToHand_calibration_url",calibrationInfoUrl);
	}

	// 仅用于眼在手上标定形式！创建一个服务client，用来请求机械臂的当前位姿，以及相应的服务消息
    ros::ServiceClient client=nh.serviceClient<robot_jaka::RobotEndPosition>("robot_service/robot_end_position");
	robot_jaka::RobotEndPosition srv;

	// 读取标定文件中保存的结果（旋转矩阵和平移向量）到对应的变量中
	cv::FileStorage calibrationFile(calibrationInfoUrl, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
	cv::Mat rotate;
	cv::Mat translation;
	calibrationFile["rotate"] >> rotate;
	calibrationFile["translation"] >> translation;
	calibrationFile.release();
	// 使用读取到的旋转矩阵和平移向量构建4×4的坐标变换矩阵
	cv::Matx44d calibrationInfo = {
		rotate.at<double>(0, 0), 	rotate.at<double>(0, 1), 	rotate.at<double>(0, 2), 	translation.at<double>(0, 0),
		rotate.at<double>(1, 0), 	rotate.at<double>(1, 1), 	rotate.at<double>(1, 2), 	translation.at<double>(1, 0),
		rotate.at<double>(2, 0), 	rotate.at<double>(2, 1), 	rotate.at<double>(2, 2), 	translation.at<double>(2, 0),
		0,							0, 							0, 							1};


	// 使用ImageTransport分别创建4个图像发布器
	// colorImagePub：原始RGB图像发布器				processImagePub：对RGB图像进行处理后的图像发布器
	// depthImagePub：原始Depth着色图像发布器		monitorImagePub：用于用户交互的图像发布器
	image_transport::ImageTransport colorImageTransport(nh);
	image_transport::ImageTransport depthImageTransport(nh);
	image_transport::ImageTransport processImageTransport(nh);
	image_transport::ImageTransport monitorImageTransport(nh);
	image_transport::Publisher colorImagePub=colorImageTransport.advertise("/vision/d435i/color",1);
	image_transport::Publisher depthImagePub=depthImageTransport.advertise("/vision/d435i/depth",1);
	image_transport::Publisher processImagePub=processImageTransport.advertise("/vision/process",1);
	image_transport::Publisher monitorImagePub=monitorImageTransport.advertise("/vision/monitor",1);
	// 为检测到的目标创建Pose发布器
	ros::Publisher targetPosePub=nh.advertise<geometry_msgs::PoseStamped>("/vision/target/pose",1);

	// 初始化Realsense相机，启动color流和depth流
    ROS_INFO("Connect to camera ......");
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR,1920,1080,RS2_FORMAT_BGR8);
	cfg.enable_stream(RS2_STREAM_DEPTH,1280,720);
	pipe.start(cfg);
	cv::waitKey(3000);
	ROS_INFO("Connected camera success!");

	// 获取相机color流的配置，并构建相机内参矩阵和畸变向量
	rs2::pipeline_profile profile = pipe.get_active_profile();
	rs2::video_stream_profile colorstreamprofile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	// rs2::stream_profile depthstreamprofile = profile.get_stream(RS2_STREAM_DEPTH);
	rs2_intrinsics intr = colorstreamprofile.get_intrinsics();
	cv::Matx33d cameraMatrix = {	intr.fx,	0,			intr.ppx,
									0,			intr.fy,	intr.ppy,
									0,			0,			1 };
	vector<double> distCoeffs(begin(intr.coeffs), end(intr.coeffs));

	// 获取相机的深度传感器，并将其设置为高精度模式
	rs2::depth_sensor sensor = profile.get_device().first<rs2::depth_sensor>();
	sensor.set_option(rs2_option::RS2_OPTION_VISUAL_PRESET, rs2_rs400_visual_preset::RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
	// 创建realsense对齐过滤器，将深度图像对齐到RGB图像
	rs2::align align2Color(RS2_STREAM_COLOR);
	// 创建realsense着色过滤器，将深度图像着色为彩色图像，有助于可视化
	rs2::colorizer colorizerDepth;


	// 目标检测数据采集，ros节点中止时退出采集
	while (ros::ok()) {
		// 从相机获取一组帧数据，并对其应用对齐过滤器，着色过滤器
		rs2::frameset frames = pipe.wait_for_frames();
		frames = frames.apply_filter(align2Color).apply_filter(colorizerDepth);
		// 从frameset中提取color帧、depth帧，着色depth帧
		rs2::video_frame colorFrame = frames.get_color_frame();
		rs2::depth_frame depthFrame = frames.get_depth_frame();
		rs2::video_frame depthColorFrame = frames.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8).as<rs2::video_frame>();
		// 将color帧和着色depth帧，转成opencv的Mat格式
		cv::Mat colorImage(colorFrame.get_height(),colorFrame.get_width(),CV_8UC3,const_cast<void *>(colorFrame.get_data()));
		cv::Mat depthColorImage(depthColorFrame.get_height(), depthColorFrame.get_width(), CV_8UC3, const_cast<void *>(depthColorFrame.get_data()));
		cvtColor(depthColorImage, depthColorImage, CV_RGB2BGR);		// 将着色depth帧从RGB颜色空间转换成BGR颜色空间，适配opencv的默认格式

		// 使用ros提供的cv_bridge接口，将color图像和着色depth图像转换成ros的Image消息，并通过上面定义的图像发布器发布到对应话题上
		// 用户可以订阅相应话题，根据需要进行处理
		sensor_msgs::ImagePtr colorImagePtr=cv_bridge::CvImage(std_msgs::Header(),"bgr8",colorImage).toImageMsg();
		sensor_msgs::ImagePtr depthImagePtr=cv_bridge::CvImage(std_msgs::Header(),"bgr8",depthColorImage).toImageMsg();
		colorImagePub.publish(colorImagePtr);
		depthImagePub.publish(depthImagePtr);

		// 在图像中进行目标（圆形）检测
		// 在color图像的拷贝中提取红色部分，方法为：对B或G通道值是R通道值两倍以上的像素，将其设置为黑色
		cv::Mat processImage = colorImage.clone();
		for (cv::MatIterator_<cv::Vec3b> it = processImage.begin<cv::Vec3b>(); it != processImage.end<cv::Vec3b>(); ++it) {
			if (2 * (*it)[0] > (*it)[2] || 2 * (*it)[1] > (*it)[2]) {
				*it = { 0,0,0 };
			} 
		}
		// 继续对图像进行腐蚀操作，并将图像从BGR颜色空间转换成GRAY颜色空间
		erode(processImage, processImage, cv::Mat(), cv::Point(-1, -1), 3);
		cvtColor(processImage, processImage, CV_BGR2GRAY);
		// 继续对图像进行Hough圆变换，将检测到的全部圆存储到circles变量中
		vector<cv::Vec3f> circles;
		HoughCircles(processImage, circles, cv::HOUGH_GRADIENT, 1.5, 100, 300, 20, 30, 70);
		// 将全部圆绘制到初始的color图像中
		for (const cv::Vec3f &circleInfo : circles) {
			circle(colorImage, cv::Point(circleInfo[0], circleInfo[1]), circleInfo[2], cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
		}
		// 将绘制圆后的color图像和着色depth图像进行加权混合，得到用于用户交互的监控图像
		cv::Mat monitorImage;
		cv::addWeighted(colorImage, 0.8, depthColorImage, 0.2, 0, monitorImage);

		// 使用ros提供的cv_bridge接口，将处理得到的图像和用户交互图像转换成ros的Image消息，并通过上面定义的图像发布器发布到对应话题上
		// 用户可以订阅相应话题，根据需要进行处理
		sensor_msgs::ImagePtr processImagePtr=cv_bridge::CvImage(std_msgs::Header(),"mono8",processImage).toImageMsg();
		sensor_msgs::ImagePtr monitorImagePtr=cv_bridge::CvImage(std_msgs::Header(),"bgr8",monitorImage).toImageMsg();
		processImagePub.publish(processImagePtr);
		monitorImagePub.publish(monitorImagePtr);

		// 如果检测到圆形，则将全部圆中第一个圆的圆心在机械臂base坐标系中的位置发布出去
		if (circles.size() > 0)
		{
			float target2CameraTranslation[3] = {0};				// 用于存储目标（此处为圆心）在相机坐标系中的空间位置
			float pixel[2] = {circles[0][0], circles[0][1]};		// 用于存储目标（此处为圆心）在图像中的像素位置
			// 使用realsense提供的rs2::depth_frame::get_distance函数, 获取目标像素在相机坐标系中的Z值大小
			// 使用realsense提供的rs2_deproject_pixel_to_point函数，将像素映射到其在相机坐标系中的空间位置
			// rs2_deproject_pixel_to_point的使用需要已知： 相机内参矩阵，目标像素，目标像素在相机坐标系中的Z值
			rs2_deproject_pixel_to_point(target2CameraTranslation, &intr, pixel, depthFrame.get_distance(circles[0][0], circles[0][1]));
			
			// 使用上面得到的目标（此处为圆心）在相机坐标系中的空间位置，构建 目标 ==> 相机坐标系 的坐标变换
			// 由于此处的目标检测并未获取姿态信息，因此将旋转矩阵部分置为单位矩阵参与运算，并无实际意义
			cv::Matx44d target2Camera = {1, 0, 0, target2CameraTranslation[0],
										 0, 1, 0, target2CameraTranslation[1],
										 0, 0, 1, target2CameraTranslation[2],
										 0, 0, 0, 1};

			// 在已知 目标 ==> 相机坐标系 的坐标变换基础上，计算 目标 ==> 机械臂base坐标系 的坐标变换target2Base
			cv::Matx44d target2Base;
			if (isEyeInHand){
				// 请求机械臂的当前位姿，得到[x, y, z, Rx, Ry, Rz]
				client.call(srv);
				boost::array<double, 6> position = srv.response.pos;

				// 由获取到的 [Rx, Ry, Rz]构造 机械臂末端 ==> 机械臂base 的旋转矩阵
				cv::Mat end2BaseRotate = eulerAnglesToRotationMatrix(cv::Vec3f(position[3], position[4], position[5]));
				// 构建 机械臂末端 ==> 机械臂base 的坐标变换，并将单位从米转化为毫米
				cv::Matx44d end2Base={
					end2BaseRotate.at<double>(0,0), end2BaseRotate.at<double>(0,1), end2BaseRotate.at<double>(0,2), position[0] * 0.001,
					end2BaseRotate.at<double>(1,0), end2BaseRotate.at<double>(1,1), end2BaseRotate.at<double>(1,2), position[1] * 0.001,
					end2BaseRotate.at<double>(2,0), end2BaseRotate.at<double>(2,1), end2BaseRotate.at<double>(2,2), position[2] * 0.001,
					0,								0,								0,								1};

				// 标定形式为眼在手上时，之前读取标定文件并构建的坐标变换矩阵应为 相机坐标系 ==> 机械臂末端坐标系
				cv::Matx44d& camera2End = calibrationInfo;
				// 使用 目标 ==> 相机坐标系 的坐标变换，左乘 相机坐标系 ==> 机械臂末端坐标系，再左乘 机械臂末端坐标系 ==> 机械臂base坐标系 的坐标变换
				// 求得 目标 ==> 机械臂base坐标系 的坐标变换
				target2Base = end2Base * camera2End * target2Camera;
			}else{
				// 标定形式为眼在手外时，之前读取标定文件并构建的坐标变换矩阵应为 相机坐标系 ==> 机械臂base坐标系
				cv::Matx44d& camera2Base = calibrationInfo;
				// 使用 目标 ==> 相机坐标系 的坐标变换，左乘 相机坐标系 ==> 机械臂base坐标系 的坐标变换
				// 求得 目标 ==> 机械臂base坐标系 的坐标变换
				target2Base = camera2Base * target2Camera;
			}
			cout << "target2Base: " << target2Base << endl;

			// 使用target2Base中的平移向量部分，构建PoseStamped消息并发布
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
		// 按照循环频率延时
		loopRate.sleep();
	}

	ROS_INFO("****** Stopped Object Detection (COLOR) Node ! ******");
	return 0;
}


	// Mat edge;
	// Canny(colorImage, edge, 50, 100, 3, true);
	// morphologyEx(colorImage, colorImage, MORPH_CLOSE,Mat(),Point(-1,-1),3);
	// cvtColor(colorImage, colorImage, CV_BGR2GRAY);
	// vector<vector<Point>> t;
	// findContours(colorImage, t, cv::noArray(), RETR_LIST, CHAIN_APPROX_SIMPLE);
	// double maxArea = 0;
	// vector<Point> *tc;
	// for (auto &curve : t) {
	// 	double curveArea = contourArea(curve);
	// 	if(curveArea>maxArea){
	// 		maxArea = curveArea;
	// 		tc = &curve;
	// 	}
	// }
	// rectangle(colorImage, boundingRect(*tc), Scalar(100, 0, 0));