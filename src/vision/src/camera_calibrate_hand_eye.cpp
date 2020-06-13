#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include "assist_tools.h"
#include "robot_jaka/RobotEndPosition.h"

using namespace std;

int main(int argc, char **argv) {
	// 初始化ros节点，创建节点句柄
    ros::init(argc, argv, "camera_calibrate_hand_eye");
    ros::NodeHandle nh;
	ROS_INFO("****** Starting Camera Calibrate Hand Eye Node ... ... ******");

	// 从ros参数服务器获取手眼标定形式（眼在手外或眼在手上），以及相应的标定文件存储路径
	// isEyeInHand == true：眼在手上		isEyeInHand == false：眼在手外	
	bool isEyeInHand;
	string calibrationInfoUrl;
	ros::param::get("camera_calibrate_hand_eye/isEyeInHand",isEyeInHand);
	if(isEyeInHand){
		ros::param::get("camera_calibrate_hand_eye/eyeInHand_calibration_url",calibrationInfoUrl);
	}else{
		ros::param::get("camera_calibrate_hand_eye/eyeToHand_calibration_url",calibrationInfoUrl);
	}

	// 创建一个服务client，用来请求机械臂的当前位姿，以及相应的服务消息
    ros::ServiceClient client=nh.serviceClient<robot_jaka::RobotEndPosition>("robot_service/robot_end_position");
	robot_jaka::RobotEndPosition srv;

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

	// 获取相机color流的配置，并构建相机内参矩阵和畸变向量
	rs2::pipeline_profile profile = pipe.get_active_profile();
	rs2::video_stream_profile colorstreamprofile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	// rs2::stream_profile depthstreamprofile = profile.get_stream(RS2_STREAM_DEPTH);
	rs2_intrinsics intr=colorstreamprofile.get_intrinsics();
	cv::Matx33f cameraMatrix = {	intr.fx,	0,			intr.ppx,
									0,			intr.fy,	intr.ppy,
									0,			0,			1 };
	vector<float> distCoeffs(begin(intr.coeffs), end(intr.coeffs));


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
				// 当检测到标定板的圆圈中心时，使用opencv提供的角点绘制函数绘制相应特征到图像中，并将图像反色特殊显示
				cv::drawChessboardCorners(colorMat, cv::Size(boardWidth, boardHeight), circleCenters, findGrid);
				colorMat ^= cv::Scalar::all(0xFF);
				cv::imshow("Camera Image", colorMat);
				cv::waitKey(1000);

				// 计算 标定板坐标系 ==> 相机坐标系 的坐标变换
				cv::Mat rotateRodriguesVec;						// 标定板坐标系 ==> 相机坐标系 旋转向量（Rodrigues形式）
				cv::Mat translationVec;							// 标定板坐标系 ==> 相机坐标系 平移向量
				// 使用opencv提供的相机外参计算函数，传入上面已经获取到的参数，计算标定板坐标系到相机坐标系的坐标变换，并存储到rotateRodriguesVec和translationVec
				bool hasObjectPosition = cv::solvePnP(objectPoints, circleCenters, cameraMatrix, distCoeffs, rotateRodriguesVec, translationVec);
				if (hasObjectPosition) {
					cv::Mat rotateVec;											// 标定板坐标系 ==> 相机坐标系 旋转矩阵
					cv::Rodrigues(rotateRodriguesVec, rotateVec);				// 将Rodrigues形式的旋转向量转换成旋转矩阵
					target2CameraRotateVecs.push_back(rotateVec);				// 将本次采集并计算得到的旋转矩阵计算结果存储下来，用于手眼标定的后续计算
					target2CameraTranslationVecs.push_back(translationVec);		// 将本次采集并计算得到的平移向量计算结果存储下来，用于手眼标定的后续计算
					
					cout << "target2CameraRotateVec" << endl << rotateVec << endl;
					cout << "target2CameraTranslationVec" << endl << translationVec << endl;

                    // 请求机械臂的当前位姿，得到[x, y, z, Rx, Ry, Rz]
					client.call(srv);
					boost::array<double, 6> position=srv.response.pos;

					cv::Mat end2BaseTranslationVec(3, 1, CV_64FC1);		// 机械臂末端 ==> 机械臂base  平移向量
					cv::Mat end2BaseRotateVec(3, 3, CV_64FC1);			// 机械臂末端 ==> 机械臂base  旋转矩阵
	
					// 由获取到的 [x, y, z]构造 机械臂末端 ==> 机械臂base 的平移向量，并将单位从米转化为毫米
					end2BaseTranslationVec.at<double>(0, 0) = position[0]*0.001;
					end2BaseTranslationVec.at<double>(1, 0) = position[1]*0.001;
					end2BaseTranslationVec.at<double>(2, 0) = position[2]*0.001;
					// 由获取到的 [Rx, Ry, Rz]构造 机械臂末端 ==> 机械臂base 的旋转矩阵
					end2BaseRotateVec = eulerAnglesToRotationMatrix(cv::Vec3f(position[3], position[4], position[5]));

					end2BaseTranslationVecs.push_back(end2BaseTranslationVec);	// 将本次采集到的机械臂平移向量存储下来，用于手眼标定的后续计算
					end2BaseRotateVecs.push_back(end2BaseRotateVec);			// 将本次采集到的机械臂旋转矩阵存储下来，用于手眼标定的后续计算

					cout << "end2BaseRotateVec" << endl << end2BaseRotateVec << endl;
					cout << "end2BaseTranslationVec" << endl<< end2BaseTranslationVec << endl;
					cout << "*********************************************************" << endl;
				}
			}
		}
	}
	
	// 标定数据采集结束后，当采集到的有效数据不少于4组时，进行手眼标定求解运算
	if (target2CameraTranslationVecs.size() < 4) {
		cout << "Calibration need more data！" << endl;
	} else {
		if (isEyeInHand) {
			cv::Mat camera2EndRotate;							// 相机坐标系 ==> 机械臂末端坐标系 旋转矩阵
			cv::Mat camera2EndTranslation;						// 相机坐标系 ==> 机械臂末端坐标系 平移向量
			// 使用opencv提供的手眼标定函数 void calibrateHandEye( args... ... )
			// 传入 机械臂末端 ==> 机械臂base， 标定板坐标系 ==> 相机坐标系 的已知坐标变换关系
			// 求解 相机坐标系 ==> 机械臂末端 的坐标变换关系，该变换是眼在手上标定形式的最终目标
			cv::calibrateHandEye(end2BaseRotateVecs, end2BaseTranslationVecs, target2CameraRotateVecs, target2CameraTranslationVecs, 
									camera2EndRotate, camera2EndTranslation);
			cout << "camera2EndRotate" << endl << camera2EndRotate << endl 
					<< "EulerAngles ==> " << rotationMatrixToEulerAngles(camera2EndRotate) << "<==" << endl;
			cout << "camera2EndTranslation" << endl << camera2EndTranslation << endl;

			cv::FileStorage camera2EndCalibrationFile(calibrationInfoUrl, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
			camera2EndCalibrationFile << "rotate" << camera2EndRotate;
			camera2EndCalibrationFile << "translation" << camera2EndTranslation;
			camera2EndCalibrationFile.release();
		} else {
			// 对 标定板坐标系 ==> 相机坐标系 的变换求逆运算，得到 相机坐标系 ==> 标定板坐标系 的变换，可用于眼在手外标定形式的后续计算
			vector<cv::Mat> camera2TargetRotateVecs;			// 相机坐标系 ==> 标定板坐标系 旋转矩阵
			vector<cv::Mat> camera2TargetTranslationVecs;		// 相机坐标系 ==> 标定板坐标系 平移向量
			for(int i = 0; i < target2CameraRotateVecs.size(); ++i){
				camera2TargetRotateVecs.push_back(target2CameraRotateVecs[i].inv(cv::DECOMP_SVD));
				camera2TargetTranslationVecs.push_back(-target2CameraRotateVecs[i].inv(cv::DECOMP_SVD)*target2CameraTranslationVecs[i]);
			}

			cv::Mat base2CameraRotate;							// 机械臂base坐标系 ==> 相机坐标系 旋转矩阵
			cv::Mat base2CameraTranslation;						// 机械臂base坐标系 ==> 相机坐标系 平移向量
			// 使用opencv提供的手眼标定函数 void calibrateHandEye( args... ... )
			// 传入 相机坐标系 ==> 标定板坐标系， 机械臂末端 ==> 机械臂base 的已知坐标变换关系
			// 求解 机械臂base坐标系 ==> 相机坐标系 的坐标变换关系
			cv::calibrateHandEye(camera2TargetRotateVecs, camera2TargetTranslationVecs, end2BaseRotateVecs, end2BaseTranslationVecs, 
									base2CameraRotate, base2CameraTranslation);

			// 对 机械臂base坐标系 ==> 相机坐标系 的变换求逆运算，得到 相机坐标系 ==> 机械臂base坐标系 的变换，该变换是眼在手外标定形式的最终目标
			cv::Mat camera2BaseRotate = base2CameraRotate.inv(cv::DECOMP_SVD);
			cv::Mat camera2BaseTranslation = -base2CameraRotate.inv(cv::DECOMP_SVD)*base2CameraTranslation;

			cout << "camera2BaseRotate" << endl << camera2BaseRotate << endl
					<< "EulerAngles ==> " << rotationMatrixToEulerAngles(camera2BaseRotate) << "<==" << endl;
			cout << "camera2BaseTranslation" << endl << camera2BaseTranslation << endl;

			// 保存 相机坐标系 ==> 机械臂base坐标系 标定结果到文件
			cv::FileStorage camera2BaseCalibrationFile(calibrationInfoUrl,cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
			camera2BaseCalibrationFile << "rotate" << camera2BaseRotate;
			camera2BaseCalibrationFile << "translation" << camera2BaseTranslation;
			camera2BaseCalibrationFile.release();
		}
	}

	ROS_INFO("****** Stopped Camera Calibrate Hand Eye Node ! ******");
	return 0;
}
