#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>

using namespace std;

int main(int argc, char **argv) {
	// 初始化ros节点，创建节点句柄
    ros::init(argc, argv, "camera_calibrate");
    ros::NodeHandle nh;
	ROS_INFO("****** Starting Camera Calibrate Node ... ... ******");

	// 从ros参数服务器获取相机内参的标定文件存储路径
	string calibrationInfoUrl;
    ros::param::get("camera_calibrate/intrinsic_calibration_url", calibrationInfoUrl);

	// 初始化标定板参数
	const int boardWidth = 4;					// 标定板列数
	const int boardHeight = 5;					// 标定板行数
	const double boardSquareSize = 0.025;		// 标定板网格间距，单位：米

	// 计算非对称圆网格标定板圆圈中心的坐标（以标定板左上角第一个圆圈中心为原点）
	vector<cv::Point3f> objectPoints;
	for (int row = 0; row < boardHeight; ++row) {
		for (int column = 0; column < boardWidth; ++column) {
			objectPoints.push_back(cv::Point3f((row % 2 + column * 2) * boardSquareSize, row*boardSquareSize, 0.));
		}
	}

	// 初始化Realsense相机，启动color流，无需启动depth流
    ROS_INFO("Connect to camera ......");
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR,1920,1080,RS2_FORMAT_BGR8);
	// cfg.enable_stream(RS2_STREAM_DEPTH,1280,720);
	pipe.start(cfg);
	cv::waitKey(3000);
    ROS_INFO("Connected camera success!");


	cv::namedWindow("Camera Image",cv::WINDOW_NORMAL);		// 设置图像显示窗口属性
	bool exitCollectData = false;							// 控制是否退出标定数据采集的标志
    vector<vector<cv::Point2f>> imagePointsVec;             // 相机采集信息：采集n组图像中的标定板圆心位置

	// 标定数据采集，ros节点中止时，或按下Esc键时退出采集
	while (!exitCollectData && ros::ok()) {
		// 从相机获取一组帧数据，并提取color帧，转成opencv的Mat格式，最终显示在图像显示窗口中
		rs2::frameset frames = pipe.wait_for_frames();
		rs2::video_frame colorframe = frames.get_color_frame();
		cv::Mat colorMat(colorframe.get_height(), colorframe.get_width(), CV_8UC3, const_cast<void *>(colorframe.get_data()));
		cv::imshow("Camera Image", colorMat);

		// 在50毫秒的周期内获取键盘输入，如果没有键盘输入，则直接进入下一轮数据采集
		// 如果有键盘输入，则在输入【Esc】时，退出数据采集；在输入【Enter】时，对当前图像进行处理
		int keyValue = cv::waitKey(50) & 0xFF;
		if (keyValue == 27) {									//【Esc】：将退出标定数据采集的标志设置为真，退出while循环
			exitCollectData = true;
		} else if (keyValue == 13) {							//【Enter】: 处理当前图像数据
			vector<cv::Point2f> circleCenters;					// 用于存储图像中标定板的所有圆心位置
			cv::SimpleBlobDetector::Params detectorParams;		// 用于提取黑色圆圈的参数
			detectorParams.minArea = 1000;						// 设置参数中圆圈的最小面积
			detectorParams.maxArea = 30000;						// 设置参数中圆圈的最大面积
			// opencv提供的用于查找图像中标定板的圆圈中心的函数，检测到的结果数据存放在circleCenters中，并返回true。如果未能识别，则返回false
			bool findGrid = cv::findCirclesGrid(colorMat, cv::Size(boardWidth, boardHeight), circleCenters,
				cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, cv::SimpleBlobDetector::create(detectorParams));

			if (findGrid) {
                // 当检测到标定板的圆圈中心时，将圆心位置存储下来，用于相机内参标定的计算
                imagePointsVec.push_back(circleCenters);
                // 当检测到标定板的圆圈中心时，使用opencv提供的角点绘制函数绘制相应特征到图像中，并将图像反色特殊显示
				cv::drawChessboardCorners(colorMat, cv::Size(boardWidth, boardHeight), circleCenters, findGrid);
				colorMat ^= cv::Scalar::all(0xFF);
				cv::imshow("Camera Image", colorMat);
				cv::waitKey(1000);
			}
		}
	}
	
	// 标定数据采集结束后，当采集到的有效数据不少于4组时，进行相机内参标定求解
	if (imagePointsVec.size() < 4) {
		cout << "Calibration need more data！" << endl;
	} else {
        vector<vector<cv::Point3f>> objectPointsVec(imagePointsVec.size(),objectPoints);	// n组标定板圆圈中心的坐标，每组为objectPoints的一个副本
        cv::Mat cameraMatrix;				// 用于存储相机的内参矩阵
        vector<double> distCoeffs;			// 用于存储相机的畸变系数
        vector<cv::Mat> rotateVec;			// 用于存储计算得到的n组标定板的旋转矩阵
        vector<cv::Mat> translationVec;		// 用于存储计算得到的n组标定板的平移向量

		// 使用opencv提供的相机标定函数 double calibrateCamera( args... ... )，求解相机的 内参矩阵 和 畸变系数
		// 标定函数的返回值为 重投影均方根误差
        double ret = cv::calibrateCamera(objectPointsVec, imagePointsVec, cv::Size(1920, 1080),
                            cameraMatrix, distCoeffs, rotateVec, translationVec, cv::CALIB_FIX_K5);

        cout << "RMS re-projection error: " << ret << endl;
        cout << "cameraMatrix" << endl << cameraMatrix << endl;
		cout << "distCoeffs" << endl ;
        for (const auto &e : distCoeffs){
            cout << e << ", ";
        }
        cout << endl;

		// 保存相机的 内参矩阵 和 畸变系数 标定结果到文件
        cv::FileStorage intrinsicCalibrationFile(calibrationInfoUrl, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
		intrinsicCalibrationFile << "cameraMatrix" << cameraMatrix;
		intrinsicCalibrationFile << "distCoeffs" << distCoeffs;
		intrinsicCalibrationFile.release();
	}

	ROS_INFO("****** Stopped Camera Calibrate Node ! ******");
	return 0;
}
