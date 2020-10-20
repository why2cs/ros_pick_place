#include <opencv2/opencv.hpp>
#include <vector>

// 来自 opencv-3.4.6 版本的辅助工具
// 由于 ROS kinetic 发行版中自带的 opencv-3.3.1 中没有相应函数，因此在此处引入相应实现

enum HandEyeCalibrationMethod
{
    CALIB_HAND_EYE_TSAI         = 0, //!< A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration @cite Tsai89
    CALIB_HAND_EYE_PARK         = 1, //!< Robot Sensor Calibration: Solving AX = XB on the Euclidean Group @cite Park94
    CALIB_HAND_EYE_HORAUD       = 2, //!< Hand-eye Calibration @cite Horaud95
    CALIB_HAND_EYE_ANDREFF      = 3, //!< On-line Hand-Eye Calibration @cite Andreff99
    CALIB_HAND_EYE_DANIILIDIS   = 4  //!< Hand-Eye Calibration Using Dual Quaternions @cite Daniilidis98
};

namespace cv
{
    void calibrateHandEye(InputArrayOfArrays R_gripper2base, InputArrayOfArrays t_gripper2base,
                          InputArrayOfArrays R_target2cam, InputArrayOfArrays t_target2cam,
                          OutputArray R_cam2gripper, OutputArray t_cam2gripper,
                          HandEyeCalibrationMethod method = CALIB_HAND_EYE_TSAI);
}




// 来自自定义的辅助工具

cv::Mat eulerAnglesToRotationMatrix(const cv::Vec3f &angleTheta);
cv::Vec3f rotationMatrixToEulerAngles(const cv::Mat &R);
void Tsai_HandEye(cv::Mat& Hcg, std::vector<cv::Mat>& Hgij, std::vector<cv::Mat>& Hcij);
