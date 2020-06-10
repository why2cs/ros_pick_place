#include <ros/ros.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

#include "robot_jaka/RobotEndPosition.h"
#include "calibration_handeye.cpp"


cv::Mat eulerAnglesToRotationMatrix(const cv::Vec3f &angleTheta)
{
	cv::Vec3f radianTheta = angleTheta * M_PI / 180.;

	// Calculate rotation about x axis
	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
		1,				0,				0,
		0,				cos(radianTheta[0]),	-sin(radianTheta[0]),
		0,				sin(radianTheta[0]),	cos(radianTheta[0])
		);
	// Calculate rotation about y axis
	cv::Mat R_y = (cv::Mat_<double>(3, 3) <<
		cos(radianTheta[1]),	0,				sin(radianTheta[1]),
		0,				1,				0,
		-sin(radianTheta[1]),	0,				cos(radianTheta[1])
		);
	// Calculate rotation about z axis
	cv::Mat R_z = (cv::Mat_<double>(3, 3) <<
		cos(radianTheta[2]),	-sin(radianTheta[2]), 0,
		sin(radianTheta[2]),	cos(radianTheta[2]),	0,
		0,				0,				1
		);
	// Combined rotation matrix
	// JAKA 绕基坐标系 x y z依次旋转 Rx Ry Rz角度
	cv::Mat R = R_z * R_y * R_x;
	return R;
}


cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R)
{
	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
	bool singular = sy < 1e-6; // If

	float x, y, z;
	if (!singular) {
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	} else {
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return cv::Vec3f(x, y, z) * 180. / M_PI;
}


void Tsai_HandEye(cv::Mat& Hcg, vector<cv::Mat>& Hgij, vector<cv::Mat>& Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	auto skew = [](cv::Mat mp) {
		// ......
		cv::Mat tmp = cv::Mat::zeros(cv::Size{ 3,3 }, CV_64FC1);
		tmp.at<double>(0, 1) = -mp.at<double>(2, 0);
		tmp.at<double>(1, 0) = mp.at<double>(2, 0);
		tmp.at<double>(0, 2) = mp.at<double>(1, 0);
		tmp.at<double>(2, 0) = -mp.at<double>(1, 0);
		tmp.at<double>(1, 2) = -mp.at<double>(0, 0);
		tmp.at<double>(2, 1) = mp.at<double>(0, 0);
		return tmp;
	};

	cv::Mat Pgij, Pcij;
	cv::Mat Rgij, Rcij;
	cv::Mat rgij, rcij;
	cv::Mat rngij, rncij;
	double theta_gij, theta_cij;
	cv::Mat A, b;

	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i]({ 0, 0, 3, 3 }).copyTo(Rgij);
		Hcij[i]({ 0, 0, 3, 3 }).copyTo(Rcij);

		Rodrigues(Rgij, rgij);
		Rodrigues(Rcij, rcij);

		theta_gij = norm(rgij);
		theta_cij = norm(rcij);

		rngij = rgij / theta_gij;
		rncij = rcij / theta_cij;

		Pgij = 2 * sin(theta_gij / 2)*rngij;
		Pcij = 2 * sin(theta_cij / 2)*rncij;

		A.push_back(skew(Pgij + Pcij));
		b.push_back(Pcij - Pgij);
	}

	//Compute rotation
	cv::Mat eyeM = cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat pinA = A.inv(cv::DECOMP_SVD);
	cv::Mat Pcg_prime = pinA * b;
	cv::Mat Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	cv::Mat PcgTrs = Pcg.t();
	cv::Mat Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM
		+ 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));

	//Computer Translation 
	cv::Mat AA, bb;
	cv::Mat Tgij, Tcij;
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i]({ 0, 0, 3, 3 }).copyTo(Rgij);
		Hcij[i]({ 0, 0, 3, 3 }).copyTo(Rcij);
		Hgij[i]({ 3, 0, 1, 3 }).copyTo(Tgij);
		Hcij[i]({ 3, 0, 1, 3 }).copyTo(Tcij);

		AA.push_back(Rgij - eyeM);
		bb.push_back(Rcg * Tcij - Tgij);
	}
	cv::Mat Tcg = AA.inv(cv::DECOMP_SVD) * bb;

	Hcg = cv::Mat::eye(4, 4, CV_64F);
	Rcg.copyTo(Hcg({ 0, 0, 3, 3 }));
	Tcg.copyTo(Hcg({ 3, 0, 1, 3 }));
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_calibrate_hand_eye");
    ros::NodeHandle nh;
    ros::ServiceClient client=nh.serviceClient<robot_jaka::RobotEndPosition>("robot_service/robot_end_position");
    robot_jaka::RobotEndPosition srv;
	ROS_INFO("****** Starting Camera Calibrate Hand Eye Node ... ... ******");

	//init calibrationBoard params
	const int boardWidth = 4;
	const int boardHeight = 5;
	const double boardSquareSize = 0.025;
	const bool isEyeInHand = false;

	vector<cv::Point3f> objectPoints;
	for (int row = 0; row < boardHeight; ++row) {
		for (int column = 0; column < boardWidth; ++column) {
			objectPoints.push_back(cv::Point3f((row % 2 + column * 2) * boardSquareSize, row*boardSquareSize, 0.));
		}
	}

	//load calibration file path
	string calibration_info_url;
	if(!isEyeInHand){
		ros::param::get("camera_calibrate_hand_eye/base2Camera_calibration_url",calibration_info_url);
	}else{
		ros::param::get("camera_calibrate_hand_eye/camera2End_calibration_url",calibration_info_url);
	}

	//init realsense D435, start, and construct intrinsics
    ROS_INFO("Connect to camera ......");
	rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_COLOR,1920,1080,RS2_FORMAT_BGR8);
	cfg.enable_stream(RS2_STREAM_DEPTH,1280,720);
	pipe.start(cfg);
	cv::waitKey(3000);
    ROS_INFO("Connected camera success!");


	rs2::pipeline_profile profile = pipe.get_active_profile();
	rs2::video_stream_profile colorstreamprofile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	rs2::stream_profile depthstreamprofile = profile.get_stream(RS2_STREAM_DEPTH);
	
	rs2_intrinsics intr=colorstreamprofile.get_intrinsics();
	cv::Matx33f cameraMatrix = {	intr.fx,	0,			intr.ppx,
									0,			intr.fy,	intr.ppy,
									0,			0,			1 };
	vector<float> distCoeffs(begin(intr.coeffs), end(intr.coeffs));

	cv::namedWindow("Camera Image",cv::WINDOW_NORMAL);
	bool exitCollectData = false;
	vector<cv::Mat> target2CameraRotateVecs;		//for eye in hand
	vector<cv::Mat> target2CameraTranslationVecs;	//for eye in hand
	vector<cv::Mat> camera2TargetRotateVecs;		//for eye to hand
	vector<cv::Mat> camera2TargetTranslationVecs;	//for eye to hand
	vector<cv::Mat> end2BaseRotateVecs;
	vector<cv::Mat> end2BaseTranslationVecs;


	//collectData
	while (!exitCollectData && ros::ok()) {
		rs2::frameset frames = pipe.wait_for_frames();
		rs2::video_frame colorframe = frames.get_color_frame();
		cv::Mat colorMat(colorframe.get_height(), colorframe.get_width(), CV_8UC3, const_cast<void *>(colorframe.get_data()));
		cv::imshow("Camera Image", colorMat);

		int keyValue = cv::waitKey(50) & 0xFF;
		if (keyValue == 27) {					//Esc: exit cameraStream
			exitCollectData = true;
		} else if (keyValue == 13) {			//Enter: process the frame data
			vector<cv::Point2f> circleCenters;
			cv::SimpleBlobDetector::Params detectorParams;
			detectorParams.minArea = 1000;
			detectorParams.maxArea = 30000;
			bool findGrid = cv::findCirclesGrid(colorMat, cv::Size(boardWidth, boardHeight), circleCenters,
				cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING, cv::SimpleBlobDetector::create(detectorParams));

			if (findGrid) {
				cv::drawChessboardCorners(colorMat, cv::Size(boardWidth, boardHeight), circleCenters, findGrid);
				colorMat ^= cv::Scalar::all(0xFF);
				cv::imshow("Camera Image", colorMat);
				cv::waitKey(1000);

				cv::Mat rotateRodriguesVec;
				cv::Mat translationVec;
				bool hasObjectPosition = cv::solvePnP(objectPoints, circleCenters, cameraMatrix, distCoeffs, rotateRodriguesVec, translationVec);
				if (hasObjectPosition) {
					cv::Mat rotateVec;
					cv::Rodrigues(rotateRodriguesVec, rotateVec);
					target2CameraRotateVecs.push_back(rotateVec);
					target2CameraTranslationVecs.push_back(translationVec);

					if (!isEyeInHand) {
						camera2TargetRotateVecs.push_back(rotateVec.inv(cv::DECOMP_SVD));
						camera2TargetTranslationVecs.push_back(-rotateVec.inv(cv::DECOMP_SVD)*translationVec);
					}	

					cout << "target2CameraRotateVec" << endl << rotateVec << endl;
					cout << "target2CameraTranslationVec" << endl << translationVec << endl;

                    client.call(srv);
					auto position=srv.response.pos;

					cv::Mat end2BaseTranslationVec(3, 1, CV_64FC1);
					cv::Mat end2BaseRotateVec(3, 3, CV_64FC1);
	
					end2BaseTranslationVec.at<double>(0, 0) = position[0]*0.001;
					end2BaseTranslationVec.at<double>(1, 0) = position[1]*0.001;
					end2BaseTranslationVec.at<double>(2, 0) = position[2]*0.001;
					end2BaseRotateVec = eulerAnglesToRotationMatrix(cv::Vec3f(position[3], position[4], position[5]));
					end2BaseTranslationVecs.push_back(end2BaseTranslationVec);
					end2BaseRotateVecs.push_back(end2BaseRotateVec);

					cout << "end2BaseTranslationVec" << endl<< end2BaseTranslationVec << endl;
					cout << "end2BaseRotateVec" << endl << end2BaseRotateVec << endl;
					cout << "***********************" << endl;
				}
			}
		}
	}

	if (target2CameraTranslationVecs.size() < 4) {
		cout << "need more data" << endl;
	} else {
		if (!isEyeInHand) {
			cv::Mat base2CameraRotate;
			cv::Mat base2CameraTranslation;
			cv::calibrateHandEye(camera2TargetRotateVecs, camera2TargetTranslationVecs, end2BaseRotateVecs, end2BaseTranslationVecs, base2CameraRotate, base2CameraTranslation);
			cout << "base2CameraRotate" << endl << base2CameraRotate << endl;
			cout << "base2CameraTranslation" << endl << base2CameraTranslation << endl;


			cv::Mat camera2BaseRotate = base2CameraRotate.inv(cv::DECOMP_SVD);
			cv::Mat camera2BaseTranslation = -base2CameraRotate.inv(cv::DECOMP_SVD)*base2CameraTranslation;
			cout << camera2BaseTranslation << endl << camera2BaseRotate << endl << "==> " << rotationMatrixToEulerAngles(camera2BaseRotate) << "<==" << endl;

			cv::FileStorage base2CameraCalibrationFile(calibration_info_url,cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
			base2CameraCalibrationFile << "rotate" << base2CameraRotate;
			base2CameraCalibrationFile << "translation" << base2CameraTranslation;
			base2CameraCalibrationFile.release();
		} else {
			cv::Mat camera2EndRotate;
			cv::Mat camera2EndTranslation;
			cv::calibrateHandEye(end2BaseRotateVecs, end2BaseTranslationVecs, target2CameraRotateVecs, target2CameraTranslationVecs, camera2EndRotate, camera2EndTranslation);
			cout << "camera2EndRotate" << endl << camera2EndRotate << endl << "==> " << rotationMatrixToEulerAngles(camera2EndRotate) << "<==" << endl;
			cout << "camera2EndTranslation" << endl << camera2EndTranslation << endl;

			cv::FileStorage camera2EndCalibrationFile(calibration_info_url, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
			camera2EndCalibrationFile << "rotate" << camera2EndRotate;
			camera2EndCalibrationFile << "translation" << camera2EndTranslation;
			camera2EndCalibrationFile.release();
		}
	}


	//auto sensor = profile.get_device().first();
	//sensor.set_option(rs2_option::rs2_option_visual_preset, rs2_rs400_visual_preset::rs2_rs400_visual_preset_high_accuracy);

	ROS_INFO("****** Stopped Camera Calibrate Hand Eye Node ! ******");

	return 0;
}



