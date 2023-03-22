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
#include "sensor_msgs/Imu.h"

// #include <nav_msgs/GetMap.h>
// #include <nav_msgs/SetMap.h>
// #include <nav_msgs/OccupancyGrid.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <nav_main/GetWalls.h>

#include <string>
using namespace std;

#include "ClientScope.h"

class ROSbridge
{
private:
    bool debugging;
    ros::NodeHandle* nh;

    // ros::Publisher pub;

    // Subscribers
    ros::Subscriber tcssub;
    ros::Subscriber bnoxsub;
    // TODO: Make callback and add to constructor
    ros::Subscriber limitswitch1;
    ros::Subscriber localizationsub;
    ros::Subscriber centersub;
    ros::Subscriber imusub;

    // Publishers
    ros::Publisher dispenserpub;

    ros::Publisher debugpub;

    // Service clients
    // ros::ServiceClient mapclient;
    ros::ServiceClient moveBaseMapResetClient;
    ros::ServiceClient hsMapReset;

    ros::ServiceClient wallsClient;

    // Action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;
    
    // Subscriber callbacks
    void tcscallback(const std_msgs::Char::ConstPtr& msg);
    void bnoxcallback(const std_msgs::Float64::ConstPtr& msg);
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
    void localizationCallback(const geometry_msgs::Point::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

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
    bool limitSwitch1;

    // Imu data
    double xImu;
    double yImu;
    double zImu;
    double wImu;
    // Roll, pitch, yaw
    double roll;
    double pitch;
    double yaw;

    bool started;
    double northYaw;
    double eastYaw;
    double southYaw;
    double westYaw;

    double xdistCenter;
    double ydistCenter;

    ROSbridge(ros::NodeHandle* n);

    void clearMap();
    void sendUnitGoal(int movement, int rDirection);

    vector<int> getWalls();
    void sendKit();
    int getVictims();

    void pubDebug(string msg);
};


ROSbridge::ROSbridge(ros::NodeHandle* n) : ac("move_base", true)
{
    debugging = true;
    started = false;
    xdistCenter = 0;
    ydistCenter = 0;

    ROS_INFO("Creating ROSbridge");
    nh = n;
 
    ROS_INFO("Creating subscribers");

    tcssub = nh->subscribe("/sensor/tcs", 1000, &ROSbridge::tcscallback, this);
    bnoxsub = nh->subscribe("/sensor/bno/x", 1000, &ROSbridge::bnoxcallback, this);
    centersub = nh->subscribe("/center_location", 10, &ROSbridge::localizationCallback, this);
    imusub = nh->subscribe("/imu", 10, &ROSbridge::imuCallback, this);

    ROS_INFO("Created subscribers");

    dispenserpub = nh->advertise<std_msgs::String>("/dispenser", 1000);
    debugpub = nh->advertise<std_msgs::String>("/debug", 1000);

    // mapclient = nh->serviceClient<nav_msgs::GetMap>();
    moveBaseMapResetClient = nh->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    hsMapReset = nh->serviceClient<std_srvs::Trigger>("/reset_map");

    wallsClient = nh->serviceClient<nav_main::GetWalls>("/get_walls");

    ROS_INFO("Created service clients");
}

// Subscriber callbacks

void ROSbridge::tcscallback(const std_msgs::Char::ConstPtr& msg)
{
    ROS_INFO("I heard: [%c]", msg->data);
    // tcsdata = msg->data;
    tcsdata = 'b';
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

void ROSbridge::localizationCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    ROS_INFO("LocalizationCallback: %f, %f", msg->x, msg->y);
    xdistCenter = msg->x;
    ydistCenter = msg->y;
}

void ROSbridge::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
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

        // Range: -pi to pi
        // Clockwise gives smaller values
        // Counter-clockwise gives larger values

        // // East
        // if (northYaw > 0)
        //     eastYaw = northYaw - M_PI_2;
        // else
        //     eastYaw = northYaw + M_PI_2;

        // // South
        // if (northYaw > 0)
        //     southYaw = northYaw - M_PI;
        // else
        //     southYaw = northYaw + M_PI;

        // // West
        // if (northYaw > 0)
        //     westYaw = northYaw - M_PI_2 * 3;
        // else
        //     westYaw = northYaw + M_PI_2 * 3;

        // ROS_INFO("northYaw: %f", northYaw);


    }

    if (false)
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

    // ROS_INFO("Feedback x %f", feedback->base_position.pose.position.x);
    // ROS_INFO("Feedback y %f", feedback->base_position.pose.position.y);
    // ROS_INFO("Feedback z %f", feedback->base_position.pose.position.z);

    if (tcsdata == '1')
    {
        ROS_INFO("Cancelling goal");
        ac.cancelGoal();
    }
    else if (limitSwitch1)
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

void ROSbridge::sendUnitGoal(int movement, int rDirection)
{
    ros::spinOnce();
    ROS_INFO_STREAM("handleUnitMovements: " << movement);

    geometry_msgs::Pose pose;

    if (movement == 0) // Forward
    {
        ROS_INFO("North");
        
        pose.position.x = 0.3;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        // use the center location to adjust the goal, using the direction of the robot
        pubDebug("xDist: " + std::to_string(xdistCenter) + " yDist: " + std::to_string(ydistCenter) + " rDir: " + std::to_string(rDirection));

        if (rDirection == 0) // North
        {
            pose.position.x += ydistCenter * 0.01;
            pose.position.y = xdistCenter * -0.01;
        }
        else if (rDirection == 1) // East
        {
            pose.position.x += xdistCenter * 0.01;
            pose.position.y = ydistCenter * 0.01;
        }
        else if (rDirection == 2) // South
        {
            pose.position.x += ydistCenter * -0.01;
            pose.position.y = xdistCenter * 0.01;
        }
        else if (rDirection == 3) // West
        {
            pose.position.x += xdistCenter * -0.01;
            pose.position.y = ydistCenter * -0.01;
        }
        
        pubDebug("Pose x: " + std::to_string(pose.position.x) + " y: " + std::to_string(pose.position.y));

    }
    else if (movement == 1) // Turn right
    {
        ROS_INFO("Turn right");

        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        // Calculate angle to get 90 degree turn
        


        // pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);
    }
    else if (movement == 2) // Backward
    {
        ROS_INFO("Backwards");

        pose.position.x = -0.3 + xdistCenter;
        pose.position.y = 0 + ydistCenter;
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
        // pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);

    }

    else if (movement == 4) // Turn around
    {
        // ROS_INFO("Turn around");

        pose.position.x = 0;
        pose.position.y = 0.3;
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw(0);
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

vector<int> ROSbridge::getWalls()
{
    ROS_INFO("Getting walls");

    nav_main::GetWalls walls;
    wallsClient.call(walls);

    ROS_INFO("Got walls front:%d, right: %d, back: %d, left: %d", walls.response.front, walls.response.right, walls.response.back, walls.response.left);

    vector<int> wallsVector = {walls.response.front, walls.response.right, walls.response.back, walls.response.left};

    return wallsVector;
}

void ROSbridge::sendKit()
{
    ROS_INFO("Sending kit");

    std_msgs::String msg;
    msg.data = "1";
    dispenserpub.publish(msg);
}

int ROSbridge::getVictims()
{
    return 0;
}

void ROSbridge::pubDebug(string m)
{
    if (debugging)
    {
        std_msgs::String msg;
        msg.data = m;
        debugpub.publish(msg);
    }
}