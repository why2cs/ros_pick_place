#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "robot_jaka/RobotEndMove.h"

using namespace std;

ros::ServiceClient robotEndMoveClient;
string command;

void robotMoveToPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (command == "p")
    {
        robot_jaka::RobotEndMove srv;
        srv.request.pos = {msg->pose.position.x*1000, msg->pose.position.y*1000, msg->pose.position.z*1000 + 20,
                           179, 0, 0};
        srv.request.speed = 20;
        robotEndMoveClient.call(srv);
        auto status = srv.response.status;
        if (status){
            ROS_INFO("robot moving ......");
        }else{
            ROS_INFO("robot move to pose [%f,%f,%f] failed!", msg->pose.position.x * 1000,
                     msg->pose.position.y * 1000, msg->pose.position.z * 1000 + 20);
        }
    }else if(command == "h"){
        robot_jaka::RobotEndMove srv;
        srv.request.pos = {300, 0, 400, 179, 0, 0};
        srv.request.speed = 50;
        robotEndMoveClient.call(srv);
        auto status = srv.response.status;
        if (status){
            ROS_INFO("robot moving ......");
        }
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vision_command");
    ros::NodeHandle nh;
	ROS_INFO("****** Starting Vision Command Node ... ... ******");

    robotEndMoveClient=nh.serviceClient<robot_jaka::RobotEndMove>("robot_service/robot_end_move");
    ros::Subscriber targetPoseSub=nh.subscribe<geometry_msgs::PoseStamped>("/vision/target/pose",1,robotMoveToPose);
    while(ros::ok() && cin >> command){
        ros::spinOnce();
    }

	ROS_INFO("****** Stopped Vision Command Node ! ******");
}
