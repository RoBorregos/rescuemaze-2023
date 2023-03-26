#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64.h"


class TfBroadcaster
{
public:
    TfBroadcaster(ros::NodeHandle *n);

    void publishTransform();
    void setTransformYaw(double yaw);
    void run();
    void localizationCallback(const geometry_msgs::Point::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void idealOrientationCallback(const std_msgs::Float64::ConstPtr &msg);

private:
    ros::NodeHandle *n;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    geometry_msgs::TransformStamped transformStamped;

    double transformYaw;
    bool started;
    double xdistCenter = 0.0;
    double ydistCenter = 0.0;
    ros::Subscriber centersub;
    tf::TransformListener listener_;
    ros::Subscriber imusub;
    ros::Subscriber idealorientationsub;
    // Imu data
    double xImu;
    double yImu;
    double zImu;
    double wImu;
    // Roll, pitch, yaw
    double roll;
    double pitch;
    double yaw;



    double idealOrientation = 0.0;
};

TfBroadcaster::TfBroadcaster(ros::NodeHandle *n)
{
    this->n = n;
    transformYaw = 0;
    started = false;
    centersub = n->subscribe("/center_location", 10, &TfBroadcaster::localizationCallback, this);
    imusub = n->subscribe("/imu", 10, &TfBroadcaster::imuCallback, this);
    idealorientationsub = n->subscribe("/ideal_orientation", 10, &TfBroadcaster::idealOrientationCallback, this);
}

void TfBroadcaster::localizationCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    xdistCenter = msg->x;
    ydistCenter = msg->y;
}

void TfBroadcaster::setTransformYaw(double yaw)
{
    if (!started)
    {
        started = true;
    }
    transformYaw = yaw;
    publishTransform();
}

void TfBroadcaster::publishTransform()
{
    if(listener_.canTransform("/map", "/base_link", ros::Time(0)) == false)
    {
      return;
    }
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    listener_.transformPose("/map", ros::Time(0), pose, "/base_link", pose);

    if (ydistCenter != -31) {
        pose.pose.position.y += ydistCenter * 0.01;
    }
    if (xdistCenter != -31) {
        pose.pose.position.x += xdistCenter * 0.01;
    }

    pose.pose.orientation = tf::createQuaternionMsgFromYaw(idealOrientation);
    // Log yaw displacement
    ROS_INFO("Yaw displacement: %f", idealOrientation);
    // Log quaternion to roll pitch yaw.
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
    
    listener_.transformPose("/base_link", ros::Time(0), pose, "/map", pose);

    ros::spinOnce();
    transformStamped = geometry_msgs::TransformStamped();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "perfect_position";
    transformStamped.header.stamp = ros::Time::now();

    //transformStamped.transform.translation = pose.pose.position;
    transformStamped.transform.translation.x = pose.pose.position.x;
    transformStamped.transform.translation.y = pose.pose.position.y;
    transformStamped.transform.translation.z = pose.pose.position.z;
    transformStamped.transform.rotation = pose.pose.orientation;

    
    if (false)
    {
        ROS_INFO("Publishing transform");
        ROS_INFO("Yaw is %f", transformStamped.transform.rotation.z);
        ROS_INFO("Orientation quaternion is %f, %f, %f, %f", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    }

    br.sendTransform(transformStamped);
}

void TfBroadcaster::run()
{
    ros::Rate rate(10.0);
    while (n->ok())
    {
        rate.sleep();
        publishTransform();
    }
}
void TfBroadcaster::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    return;
}
/*
void TfBroadcaster::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    xImu = msg->orientation.x;
    yImu = msg->orientation.y;
    zImu = msg->orientation.z;
    wImu = msg->orientation.w;

    // Roll, pitch, yaw
    tf::Quaternion q(xImu, yImu, zImu, wImu);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    if (!started)
    {
        started = true;
        northYaw = yaw;

        tfb.setTransformYaw(yaw);

        // Range: -pi to pi
        // Clockwise gives smaller values
        // Counter-clockwise gives larger values

        ROS_INFO("northYaw: %f", northYaw);
        // East
        // eastYaw = (northYaw * (180 / M_PI) - 180) * M_PI / 180;

        if (northYaw > -M_PI_2)
            eastYaw = northYaw - M_PI_2;
        else
            eastYaw = northYaw + M_PI_2 * 3;

        // South
        if (northYaw > 0)
            southYaw = northYaw - M_PI;
        else
            southYaw = northYaw + M_PI;
        // if (northYaw > 0)
        //     southYaw = northYaw - M_PI;
        // else
        //     southYaw = northYaw + M_PI;

        // West
        if (northYaw > M_PI_2)
            westYaw = northYaw - M_PI_2 * 3;
        else
            westYaw = northYaw + M_PI_2;
        // if (northYaw > 0)
        //     westYaw = northYaw - M_PI_2 * 3;
        // else
        //     westYaw = northYaw + M_PI_2 * 3;
    }

    if (true)
    {
        // ROS_INFO("x: %f", xImu);
        // ROS_INFO("y: %f", yImu);
        // ROS_INFO("z: %f", zImu);
        // ROS_INFO("w: %f", wImu);
        // ROS_INFO("roll: %f", roll);
        // ROS_INFO("pitch: %f", pitch);
        ROS_INFO("yaw: %f", yaw);
    }
}
*/

void TfBroadcaster::idealOrientationCallback(const std_msgs::Float64::ConstPtr &msg)
{
    idealOrientation = msg->data;
    ROS_INFO("Ideal orientation: %f", idealOrientation);
}