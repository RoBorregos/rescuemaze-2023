#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>

#include <move_base_msgs/MoveBaseAction.h>
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

// #include <nav_msgs/GetMap.h>
// #include <nav_msgs/SetMap.h>
// #include <nav_msgs/OccupancyGrid.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <string>
using namespace std;

#include "ClientScope.h"

class ROSbridge
{
private:
    ros::NodeHandle* nh;
    // ros::Publisher pub;

    // Subscribers
    ros::Subscriber tcssub;
    ros::Subscriber bnoxsub;

    // Service clients
    ros::ServiceClient mapclient;
    ros::ServiceClient moveBaseMapResetClient;
    ros::ServiceClient hsMapReset;

    // Action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    
    // Subscriber callbacks
    void tcscallback(const std_msgs::Char::ConstPtr& msg);
    void bnoxcallback(const std_msgs::Float64::ConstPtr& msg);
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    // Action client callbacks
    void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result);
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    void activeCb();

    // Action client functions
    void movementClientAsync(move_base_msgs::MoveBaseGoal goal);
    void sendGoal(geometry_msgs::Pose pose);

    void cancelGoal();

public:
    ClientScope scope;
    char tcsdata;
    float bnoxdata;

    ROSbridge(ros::NodeHandle* n);

    void clearMap();
    void sendUnitGoal(int movement);

};


ROSbridge::ROSbridge(ros::NodeHandle* n) : ac("move_base", true)
{
    ROS_INFO("Creating ROSbridge");
    nh = n;
 
    ROS_INFO("Creating subscribers");

    tcssub = nh->subscribe("/sensor/tcs", 1000, &ROSbridge::tcscallback, this);
    bnoxsub = nh->subscribe("/sensor/bno/x", 1000, &ROSbridge::bnoxcallback, this);

    ROS_INFO("Created subscribers");

    // mapclient = nh->serviceClient<nav_msgs::GetMap>();
    moveBaseMapResetClient = nh->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    hsMapReset = nh->serviceClient<std_srvs::Trigger>("/reset_map");

    ROS_INFO("Created service clients");
}

// Subscriber callbacks

void ROSbridge::tcscallback(const std_msgs::Char::ConstPtr& msg)
{
    ROS_INFO("I heard: [%c]", msg->data);
    tcsdata = msg->data;
}

void ROSbridge::bnoxcallback(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_INFO("BNO x: %f", msg->data);
    bnoxdata = msg->data;
}

void ROSbridge::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    ROS_INFO("StatusCallback: %d", msg->status_list[0].status);
    if (scope.startedGoal)
        scope.status = msg->status_list[0].status;
}

void ROSbridge::doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
    ROS_INFO_STREAM("done callback: Finished in state " << state.toString());
    // ROS_INFO_STREAM("Answer: " << result->sequence.back());

    scope.result = state.state_;
    scope.textRes = state.getText();

    // scope.result = result->status.goal_id.id;
    scope.resultReceived = true;
    scope.startedGoal = false;
}

void ROSbridge::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{

    ROS_INFO("Feedback x %f", feedback->base_position.pose.position.x);
    ROS_INFO("Feedback y %f", feedback->base_position.pose.position.y);
    ROS_INFO("Feedback z %f", feedback->base_position.pose.position.z);

    if (tcsdata == '1')
    {
        ROS_INFO("Cancelling goal");
        ac.cancelGoal();
    }

    // scope.feedback = feedback->base_position.pose;
    scope.xfeedback = feedback->base_position.pose.position.x;
    scope.yfeedback = feedback->base_position.pose.position.y;
    scope.zfeedback = feedback->base_position.pose.position.z;
}

void ROSbridge::activeCb()
{
    ROS_INFO("Goal just went active");
    scope.resultReceived = false;
    scope.startedGoal = true;
}

// Action client functions

void ROSbridge::movementClientAsync(move_base_msgs::MoveBaseGoal goal)
{
    ROS_INFO("Sending goal");

    ac.waitForServer();

    ac.sendGoal(goal, boost::bind(&ROSbridge::doneCb, this, _1, _2), boost::bind(&ROSbridge::activeCb, this), boost::bind(&ROSbridge::feedbackCb, this, _1));

    ac.waitForResult();

    ROS_INFO("Finished goal");

    scope.resultReceived = true;
}

void ROSbridge::sendGoal(geometry_msgs::Pose pose)
{
    move_base_msgs::MoveBaseGoal goal;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "base_link";
    poseStamped.pose = pose;
    
    goal.target_pose = poseStamped;
    movementClientAsync(goal);
}

void ROSbridge::sendUnitGoal(int movement)
{
    ROS_INFO_STREAM("handleUnitMovements: " << movement);

    geometry_msgs::Pose pose;

    if (movement == 0) // North
    {
        ROS_INFO("North");

        pose.position.x = 0.3;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
    }
    else if (movement == 1) // Turn right
    {
        ROS_INFO("Turn right");

        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);
    }
    else if (movement == 2) // South
    {
        ROS_INFO("Backwards");

        pose.position.x = -0.3;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;
    }
    else if (movement == 3) // Turn left
    {
        ROS_INFO("Turn left");

        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
    }

    sendGoal(pose);
}

void ROSbridge::cancelGoal()
{
    ac.cancelAllGoals();
}

void ROSbridge::clearMap()
{
    ROS_INFO("Clearing map");

    std_srvs::Empty empty;
    std_srvs::Trigger trigger;

    moveBaseMapResetClient.call(empty);
    hsMapReset.call(trigger);

    ROS_INFO("Cleared map");
}