#include <opencv2/opencv.hpp>
#include "assist_tools.h"

using namespace std;
using namespace cv;

Mat eulerAnglesToRotationMatrix(const Vec3f &angleTheta)
{
    Vec3f radianTheta = angleTheta * M_PI / 180.;

    // Calculate rotation about x axis
    Mat R_x = (Mat_<double>(3, 3) <<
        1,				0,				0,
        0,				cos(radianTheta[2]),	-sin(radianTheta[2]),
        0,				sin(radianTheta[2]),	cos(radianTheta[2])
        );
    // Calculate rotation about y axis
    Mat R_y = (Mat_<double>(3, 3) <<
        cos(radianTheta[1]),	0,				sin(radianTheta[1]),
        0,				1,				0,
        -sin(radianTheta[1]),	0,				cos(radianTheta[1])
        );
    // Calculate rotation about z axis
    Mat R_z = (Mat_<double>(3, 3) <<
        cos(radianTheta[0]),	-sin(radianTheta[0]),   0,
        sin(radianTheta[0]),	cos(radianTheta[0]),    0,
        0,				0,				1
        );
    // Combined rotation matrix
    // 自主机械臂是基于基坐标系依次绕 z y x轴旋转r p y角度
    return R_x * R_y * R_z;
}


Vec3f rotationMatrixToEulerAngles(const Mat &R)
{
    double r, p, y;
    if(abs(abs(R.at<double>(0,2))-1)<1e-6)
    {
        r=0;
        if(R.at<double>(0,2)>0){
            y=atan2(R.at<double>(2,1),R.at<double>(1,1));
        }else{
            y=-atan2(R.at<double>(1,0),R.at<double>(2,0));
        }
        p=asin(R.at<double>(0,2));
    }else{
        r=-atan2(R.at<double>(0,1),R.at<double>(0,0));
        y=-atan2(R.at<double>(1,2),R.at<double>(2,2));
        p=atan(R.at<double>(0,2)*cos(r)/R.at<double>(0,0));
    }

    return Vec3d(r, p, y) * 180. / M_PI;
}


// 弃用，供参考
void Tsai_HandEye(Mat& Hcg, vector<Mat>& Hgij, vector<Mat>& Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nStatus = Hgij.size();

	auto skew = [](Mat mp) {
		// ......
		Mat tmp = Mat::zeros(Size{ 3,3 }, CV_64FC1);
		tmp.at<double>(0, 1) = -mp.at<double>(2, 0);
		tmp.at<double>(1, 0) = mp.at<double>(2, 0);
		tmp.at<double>(0, 2) = mp.at<double>(1, 0);
		tmp.at<double>(2, 0) = -mp.at<double>(1, 0);
		tmp.at<double>(1, 2) = -mp.at<double>(0, 0);
		tmp.at<double>(2, 1) = mp.at<double>(0, 0);
		return tmp;
	};

	Mat Pgij, Pcij;
	Mat Rgij, Rcij;
	Mat rgij, rcij;
	Mat rngij, rncij;
	double theta_gij, theta_cij;
	Mat A, b;

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
	Mat eyeM = Mat::eye(3, 3, CV_64FC1);
	Mat pinA = A.inv(DECOMP_SVD);
	Mat Pcg_prime = pinA * b;
	Mat Pcg = 2 * Pcg_prime / sqrt(1 + norm(Pcg_prime) * norm(Pcg_prime));
	Mat PcgTrs = Pcg.t();
	Mat Rcg = (1 - norm(Pcg) * norm(Pcg) / 2) * eyeM
		+ 0.5 * (Pcg * PcgTrs + sqrt(4 - norm(Pcg)*norm(Pcg))*skew(Pcg));

	//Computer Translation 
	Mat AA, bb;
	Mat Tgij, Tcij;
	for (int i = 0; i < nStatus; i++)
	{
		Hgij[i]({ 0, 0, 3, 3 }).copyTo(Rgij);
		Hcij[i]({ 0, 0, 3, 3 }).copyTo(Rcij);
		Hgij[i]({ 3, 0, 1, 3 }).copyTo(Tgij);
		Hcij[i]({ 3, 0, 1, 3 }).copyTo(Tcij);

		AA.push_back(Rgij - eyeM);
		bb.push_back(Rcg * Tcij - Tgij);
	}
	Mat Tcg = AA.inv(DECOMP_SVD) * bb;

	Hcg = Mat::eye(4, 4, CV_64F);
	Rcg.copyTo(Hcg({ 0, 0, 3, 3 }));
	Tcg.copyTo(Hcg({ 3, 0, 1, 3 }));
}
