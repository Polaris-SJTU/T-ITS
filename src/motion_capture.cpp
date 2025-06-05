#include <ros/ros.h>
#include <unordered_map>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include "LuMoSDKBase.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_capture");
    ros::NodeHandle nh;

    std::unordered_map<std::string, ros::Publisher> pose_publishers;
    std::unordered_map<std::string, ros::Publisher> twist_publishers;

    std::string server;
    nh.getParam("server", server);

    std::shared_ptr<lusternet::CReceiveBase> LusterMotionData = lusternet::getFZReceive();
    LusterMotionData->Init();
    LusterMotionData->Connect(server);

    lusternet::LusterMocapData MocapData;
    while (ros::ok) {
        if (LusterMotionData->IsConnected()) {
            LusterMotionData->ReceiveData(MocapData);

            std::vector<lusternet::LST_RIGID_DATA> FrameRigidBody = MocapData.FrameRigidBody;
            for (lusternet::LST_RIGID_DATA RigidData : FrameRigidBody) {
                std::string RigidName = RigidData.RigidName;

                if (pose_publishers.find(RigidName) == pose_publishers.end()) {
                    pose_publishers[RigidName] = nh.advertise<geometry_msgs::Pose>(RigidName + "/pose", 10);
                    twist_publishers[RigidName] = nh.advertise<geometry_msgs::Twist>(RigidName + "/twist", 10);
                }

                if (RigidData.IsTrack) {
                    geometry_msgs::Pose pose;
                    pose.position.x = RigidData.Z / 1000.0;
                    pose.position.y = RigidData.X / 1000.0;
                    pose.position.z = RigidData.Y / 1000.0;
                    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(
                        RigidData.fZEulerAngle / 180.0 * M_PI,
                        RigidData.fXEulerAngle / 180.0 * M_PI,
                        RigidData.fYEulerAngle / 180.0 * M_PI
                    );
                    pose.orientation = quaternion;
                    pose_publishers[RigidName].publish(pose);

                    geometry_msgs::Twist twist;
                    twist.linear.x = RigidData.fZSpeed;
                    twist.linear.y = RigidData.fXSpeed;
                    twist.linear.z = RigidData.fYSpeed;
                    twist.angular.x = RigidData.fZPalstance;
                    twist.angular.y = RigidData.fXPalstance;
                    twist.angular.z = RigidData.fYPalstance;
                    twist_publishers[RigidName].publish(twist);
                }
            }
        }
    }

    if (LusterMotionData->IsConnected()) {
        LusterMotionData->Disconnect(server);
    }
    LusterMotionData->Close();
    
    return 0;
}