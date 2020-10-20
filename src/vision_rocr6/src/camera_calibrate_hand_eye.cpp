#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <iostream>
#include "assist_tools.h"
#include "rocr6_msgs/Feedback.h"

using namespace std;

int main(int argc, char **argv) {
	// 初始化ros节点，创建节点句柄
    ros::init(argc, argv, "camera_calibrate_hand_eye");
    ros::NodeHandle nh;
	ROS_INFO("==========>START Camera Hand Eye Calibration, press keys on IMAGE WINDOW: [Enter]:add data [Esc]:exit<==========");
	
	// 从ros参数服务器获取手眼标定形式（眼在手外或眼在手上），以及相应的标定文件存储路径
	// isEyeInHand == true：眼在手上		isEyeInHand == false：眼在手外	
	bool isEyeInHand;							// 手眼标定形式
	string cameraIntrinsicCalibrationUrl;		// 相机内参标定文件路径
	string calibrationInfoUrl;					// 手眼标定文件路径
	ros::param::get("camera_calibrate_hand_eye/isEyeInHand",isEyeInHand);
	ros::param::get("camera_calibrate_hand_eye/camera_intrinsic_calibration_url",cameraIntrinsicCalibrationUrl);
	if(isEyeInHand){
		ros::param::get("camera_calibrate_hand_eye/eyeInHand_calibration_url",calibrationInfoUrl);
	}else{
		ros::param::get("camera_calibrate_hand_eye/eyeToHand_calibration_url",calibrationInfoUrl);
	}
	ROS_INFO("Camera Fix Type: [ %s ]",isEyeInHand?"Eye In Hand":"Eye To Hand");

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

	try{
		// 读取标定文件中保存的结果（相机内参矩阵和畸变系数）到对应的变量中
		cv::FileStorage cameraIntrinsicCalibrationFile(cameraIntrinsicCalibrationUrl, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
		cv::Mat cameraMatrix;
		vector<double> distCoeffs;
		cameraIntrinsicCalibrationFile["cameraMatrix"] >> cameraMatrix;
		cameraIntrinsicCalibrationFile["distCoeffs"] >> distCoeffs;
		cameraIntrinsicCalibrationFile.release();
		if(cameraMatrix.empty() || distCoeffs.empty()){
			throw runtime_error("program can't load camera intrinsic, please finish the Camera Intrinsic Calibration first!");
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
		vector<cv::Mat> target2CameraRotateVecs;				// 相机采集信息：	标定板坐标系 ==> 相机坐标系 旋转矩阵
		vector<cv::Mat> target2CameraTranslationVecs;			// 相机采集信息：	标定板坐标系 ==> 相机坐标系 平移向量
		vector<cv::Mat> end2BaseRotateVecs;						// 机械臂信息：		机械臂末端 ==> 机械臂base  旋转矩阵
		vector<cv::Mat> end2BaseTranslationVecs;				// 机械臂信息：		机械臂末端 ==> 机械臂base  平移向量
		bool exitCollectData = false;							// 控制是否退出标定数据采集的标志

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
					// 计算 标定板坐标系 ==> 相机坐标系 的坐标变换
					cv::Mat rotateRodriguesVec;						// 标定板坐标系 ==> 相机坐标系 旋转向量（Rodrigues形式）
					cv::Mat translationVec;							// 标定板坐标系 ==> 相机坐标系 平移向量
					// 使用opencv提供的相机外参计算函数，传入上面已经获取到的参数，计算标定板坐标系到相机坐标系的坐标变换，并存储到rotateRodriguesVec和translationVec
					bool hasObjectPosition = cv::solvePnP(objectPoints, circleCenters, cameraMatrix, distCoeffs, rotateRodriguesVec, translationVec);
					if (hasObjectPosition) {
						vector<cv::Point2f> projectImagePoints;		// 用于存储标定板圆心在相机图像中的重投影位置
						// 使用opencv提供的相机投影图像计算函数，计算标定板圆心在相机图像中的重投影位置，并计算重投影均方根误差
						cv::projectPoints(objectPoints, rotateRodriguesVec, translationVec, cameraMatrix, distCoeffs, projectImagePoints);
						double projectOffsetPowSum = 0;
						for (size_t i = 0; i < projectImagePoints.size(); ++i){
							projectOffsetPowSum += static_cast<double>(pow(projectImagePoints[i].x - circleCenters[i].x, 2)
																	+ pow(projectImagePoints[i].y - circleCenters[i].y, 2));
						}
						double projectRMS = sqrt(projectOffsetPowSum / projectImagePoints.size());

						// 当重投影均方根误差在可接受范围内时，本次采集为有效数据，并进一步处理数据
						if (projectRMS < 1.0){
							// 获取机械臂的当前位姿，得到机械臂末端到机械臂base的坐标变换
							boost::shared_ptr<rocr6_msgs::Feedback const> msgPtr = ros::topic::waitForMessage<rocr6_msgs::Feedback>("/rocr6_msgs/feedback",nh,ros::Duration(2.));
							if (msgPtr != nullptr){
								cv::Mat end2BaseTranslationVec(3, 1, CV_64FC1); // 机械臂末端 ==> 机械臂base  平移向量
								cv::Mat end2BaseRotateVec(3, 3, CV_64FC1);		// 机械臂末端 ==> 机械臂base  旋转矩阵

								// 构造 机械臂末端 ==> 机械臂base 的平移向量，单位:米
								end2BaseTranslationVec.at<double>(0, 0) = msgPtr->homogeneous.X[3];
								end2BaseTranslationVec.at<double>(1, 0) = msgPtr->homogeneous.Y[3];
								end2BaseTranslationVec.at<double>(2, 0) = msgPtr->homogeneous.Z[3];
								// 构造 机械臂末端 ==> 机械臂base 的旋转矩阵
								end2BaseRotateVec.at<double>(0, 0) = msgPtr->homogeneous.X[0];
								end2BaseRotateVec.at<double>(0, 1) = msgPtr->homogeneous.X[1];
								end2BaseRotateVec.at<double>(0, 2) = msgPtr->homogeneous.X[2];
								end2BaseRotateVec.at<double>(1, 0) = msgPtr->homogeneous.Y[0];
								end2BaseRotateVec.at<double>(1, 1) = msgPtr->homogeneous.Y[1];
								end2BaseRotateVec.at<double>(1, 2) = msgPtr->homogeneous.Y[2];
								end2BaseRotateVec.at<double>(2, 0) = msgPtr->homogeneous.Z[0];
								end2BaseRotateVec.at<double>(2, 1) = msgPtr->homogeneous.Z[1];
								end2BaseRotateVec.at<double>(2, 2) = msgPtr->homogeneous.Z[2];
								end2BaseTranslationVecs.push_back(end2BaseTranslationVec); // 将本次采集到的机械臂平移向量存储下来，用于手眼标定的后续计算
								end2BaseRotateVecs.push_back(end2BaseRotateVec);		   // 将本次采集到的机械臂旋转矩阵存储下来，用于手眼标定的后续计算

								cv::Mat rotateVec;											// 标定板坐标系 ==> 相机坐标系 旋转矩阵
								cv::Rodrigues(rotateRodriguesVec, rotateVec);				// 将Rodrigues形式的旋转向量转换成旋转矩阵
								target2CameraRotateVecs.push_back(rotateVec);				// 将本次采集并计算得到的旋转矩阵计算结果存储下来，用于手眼标定的后续计算
								target2CameraTranslationVecs.push_back(translationVec);		// 将本次采集并计算得到的平移向量计算结果存储下来，用于手眼标定的后续计算

								ROS_INFO("The image has been checked and added (RMS re-projection error: %f) ! [%d] sets of data have been added...", projectRMS, target2CameraTranslationVecs.size());
								ROS_INFO_STREAM("calibrationBoard => camera Rotate: " << rotateVec << endl
												<< "calibrationBoard => camera Translation: " << translationVec << endl
												<< "robotEnd => robotBase Rotate: " << end2BaseRotateVec << endl
												<< "robotEnd => robotBase Translation: " << end2BaseTranslationVec);
							}else{
								ROS_WARN("Robot Pose get Failed !");
							}
						}else{
							ROS_WARN("The image has been checked and deprecated (RMS re-projection error: %f, greater than the standard value) !", projectRMS);
						}
					}
					// 当检测到标定板的圆圈中心时，使用opencv提供的角点绘制函数绘制相应特征到图像中，并将图像反色特殊显示
					cv::drawChessboardCorners(colorMat, cv::Size(boardWidth, boardHeight), circleCenters, findGrid);
					colorMat ^= cv::Scalar::all(0xFF);
					cv::imshow("Camera Image", colorMat);
					cv::waitKey(1000);
				}
			}
		}

		cv::destroyWindow("Camera Image");
		pipe.stop();

		// 标定数据采集结束后，当采集到的有效数据不少于4组时，进行手眼标定求解运算
		if (target2CameraTranslationVecs.size() < 4) {
			ROS_WARN("Calibration need more data !");
		} else {
			if (isEyeInHand) {
				cv::Mat camera2EndRotate;							// 相机坐标系 ==> 机械臂末端坐标系 旋转矩阵
				cv::Mat camera2EndTranslation;						// 相机坐标系 ==> 机械臂末端坐标系 平移向量
				// 使用opencv提供的手眼标定函数 void calibrateHandEye( args... ... )
				// 传入 机械臂末端 ==> 机械臂base， 标定板坐标系 ==> 相机坐标系 的已知坐标变换关系
				// 求解 相机坐标系 ==> 机械臂末端 的坐标变换关系，该变换是眼在手上标定形式的最终目标
				cv::calibrateHandEye(end2BaseRotateVecs, end2BaseTranslationVecs, target2CameraRotateVecs, target2CameraTranslationVecs, 
										camera2EndRotate, camera2EndTranslation);

				// 保存 相机坐标系 ==> 机械臂末端坐标系 标定结果到文件
				cv::FileStorage camera2EndCalibrationFile(calibrationInfoUrl, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
				camera2EndCalibrationFile << "rotate" << camera2EndRotate;
				camera2EndCalibrationFile << "translation" << camera2EndTranslation;
				camera2EndCalibrationFile.release();

				ROS_INFO_STREAM("camera hand eye calibration finished, and the result has been saved as: " << calibrationInfoUrl << endl
								<< "camera => robotEnd Rotate: " << camera2EndRotate << endl
								<< "camera => robotEnd Euler Angles: " << rotationMatrixToEulerAngles(camera2EndRotate) << endl
								<< "camera => robotEnd Translation: " << camera2EndTranslation);
			} else {
				// 对 标定板坐标系 ==> 相机坐标系 的变换求逆运算，得到 相机坐标系 ==> 标定板坐标系 的变换，可用于眼在手外标定形式的后续计算
				vector<cv::Mat> camera2TargetRotateVecs;			// 相机坐标系 ==> 标定板坐标系 旋转矩阵
				vector<cv::Mat> camera2TargetTranslationVecs;		// 相机坐标系 ==> 标定板坐标系 平移向量
				for(size_t i = 0; i < target2CameraTranslationVecs.size(); ++i){
					camera2TargetRotateVecs.push_back(target2CameraRotateVecs[i].t());
					camera2TargetTranslationVecs.push_back(-target2CameraRotateVecs[i].t()*target2CameraTranslationVecs[i]);
				}

				cv::Mat base2CameraRotate;							// 机械臂base坐标系 ==> 相机坐标系 旋转矩阵
				cv::Mat base2CameraTranslation;						// 机械臂base坐标系 ==> 相机坐标系 平移向量
				// 使用opencv提供的手眼标定函数 void calibrateHandEye( args... ... )
				// 传入 相机坐标系 ==> 标定板坐标系， 机械臂末端 ==> 机械臂base 的已知坐标变换关系
				// 求解 机械臂base坐标系 ==> 相机坐标系 的坐标变换关系
				cv::calibrateHandEye(camera2TargetRotateVecs, camera2TargetTranslationVecs, end2BaseRotateVecs, end2BaseTranslationVecs, 
										base2CameraRotate, base2CameraTranslation);

				// 对 机械臂base坐标系 ==> 相机坐标系 的变换求逆运算，得到 相机坐标系 ==> 机械臂base坐标系 的变换，该变换是眼在手外标定形式的最终目标
				cv::Mat camera2BaseRotate = base2CameraRotate.t();
				cv::Mat camera2BaseTranslation = -base2CameraRotate.t()*base2CameraTranslation;

				// 保存 相机坐标系 ==> 机械臂base坐标系 标定结果到文件
				cv::FileStorage camera2BaseCalibrationFile(calibrationInfoUrl,cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
				camera2BaseCalibrationFile << "rotate" << camera2BaseRotate;
				camera2BaseCalibrationFile << "translation" << camera2BaseTranslation;
				camera2BaseCalibrationFile.release();

				ROS_INFO_STREAM("camera hand eye calibration finished, and the result has been saved as: " << calibrationInfoUrl << endl
								<< "camera => robotBase Rotate: " << camera2BaseRotate << endl
								<< "camera => robotBase Euler Angles: " << rotationMatrixToEulerAngles(camera2BaseRotate) << endl
								<< "camera => robotBase Translation: " << camera2BaseTranslation);
			}
		}
	}catch (const rs2::error &e){
		ROS_FATAL("RealSense error calling %s (%s): %s", e.get_failed_function().c_str(), e.get_failed_args().c_str(), e.what());
	}
	catch (const std::exception &e){
		ROS_FATAL(e.what());
	}

	ROS_INFO("==========>Camera Hand Eye Calibration STOPPED!<==========");
	return 0;
}

