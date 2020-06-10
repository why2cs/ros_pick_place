#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <librealsense2/rsutil.h>

using namespace std;
using namespace cv;

image_transport::Publisher imagePub;

void imageCallback(const sensor_msgs::ImageConstPtr &colorMsg,const sensor_msgs::ImageConstPtr &depthMsg)
{
    cv_bridge::CvImagePtr colorImagePtr;
    cv_bridge::CvImagePtr depthImagePtr;
    try
    {
        colorImagePtr = cv_bridge::toCvCopy(colorMsg, sensor_msgs::image_encodings::BGR8);
        depthImagePtr = cv_bridge::toCvCopy(depthMsg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat colorImage=colorImagePtr->image;
    Mat processImage = colorImage.clone();
    for (MatIterator_<Vec3b> it = processImage.begin<Vec3b>(); it != processImage.end<Vec3b>(); ++it)
    {
        if (2 * (*it)[0] > (*it)[2] || 2 * (*it)[1] > (*it)[2])
        {
            *it = {0, 0, 0};
        }
    }
    erode(processImage, processImage, Mat(), Point(-1, -1), 3);
    cvtColor(processImage, processImage, CV_BGR2GRAY);

    vector<Vec3f> circles;
    HoughCircles(processImage, circles, HOUGH_GRADIENT, 1, 100, 100, 15, 10, 50);
    for (auto &&circleInfo : circles)
    {
        circle(colorImage, Point(circleInfo[0], circleInfo[1]), circleInfo[2], Scalar(0, 255, 0), 2, LINE_AA);
    }

    imshow("Image Processing...",colorImage);
    waitKey(10);
    sensor_msgs::ImagePtr pubMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImage).toImageMsg();
    imagePub.publish(pubMsg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_process");
    ros::NodeHandle nh;
    ros::Rate loopRate(6);
    

    string colorImageInputTopic;
    string depthImageInputTopic;
    string imageOutputTopic;

    // ros::param::get("visionInput",imageInputTopic);
    // ros::param::get("visionOutput",imageOutputTopic);
    colorImageInputTopic="/vision_cam_D435i/color/image_raw";
    depthImageInputTopic="/vision_cam_D435i/aligned_depth_to_color/image_raw";
    imageOutputTopic="/fuck";


    using MySyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;

    image_transport::ImageTransport imageTransport(nh);
    message_filters::Subscriber<sensor_msgs::Image> colorImageSub(nh,colorImageInputTopic,1);
    message_filters::Subscriber<sensor_msgs::Image> depthImageSub(nh,depthImageInputTopic,1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), colorImageSub, depthImageSub);
    sync.registerCallback(imageCallback);

    imagePub=imageTransport.advertise(imageOutputTopic,1);

    namedWindow("Image Processing...",WINDOW_NORMAL);





    while(ros::ok()){
        ros::spinOnce();
        loopRate.sleep();
    }


    destroyWindow("Image Processing...");
    ROS_INFO("Stoped image process node!");
    return 0;
}