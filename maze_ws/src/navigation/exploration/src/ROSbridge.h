#pragma once

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32MultiArray.h>

#include <move_base_msgs/MoveBaseAction.h>
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <tf/transform_listener.h>

// #include <nav_msgs/GetMap.h>
// #include <nav_msgs/SetMap.h>
// #include <nav_msgs/OccupancyGrid.h>

#include <ros/callback_queue.h>
#include <chrono>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <nav_main/GetWalls.h>
#include <nav_main/GetWallsDist.h>
#include <openmv_camera/BothCameras.h>

#include <exploration/GoalStatus.h>
#include <exploration/VLXDist.h>

// #include "TfBroadcaster.h"

#include <string>
#include <iostream>
using namespace std;

#include "ClientScope.h"

#define sendMapGoals true
// #define debugmapgoals true

#ifdef simulateRos

#include <map>
#include "Map.h"

#endif

class ROSbridge
{
private:
    bool debugging;

    ros::NodeHandle *nh;

#ifdef useNavStack
    bool debugmapgoals;

    double NOSEUP_PITCH;
    double NOSEDOWN_PITCH;

    geometry_msgs::TransformStamped transformStamped;
    tf::StampedTransform baselink_mapTransform;

    bool blueTile;
    bool blackTile;
    bool silverTile;

    bool downRamp;

    bool obstacle;

    bool restartGoal;

    // Timer
    ros::Timer timer;

    bool startedImu;
    double northYaw;
    double eastYaw;
    double southYaw;
    double westYaw;

    bool limitSwitchLeft;
    bool limitSwitchRight;

    // Imu data
    double xImu;
    double yImu;
    double zImu;
    double wImu;
    // Roll, pitch, yaw
    double roll;
    double pitch;
    double yaw;

    double transformYaw;

    double xdistCenter;
    double ydistCenter;

    // Subscribers
    ros::Subscriber tcssub;
    // TODO: Make callback and add to constructor
    ros::Subscriber limitswitchleftsub;
    ros::Subscriber limitswitchrightsub;
    ros::Subscriber localizationsub;
    ros::Subscriber centersub;
    ros::Subscriber imusub;

    ros::Publisher orientationpub;

    // Twist Publisher
    ros::Publisher twistpub;
    ros::Publisher pitchpub;
    ros::Publisher yawpub;

    // Service clients
    // ros::ServiceClient mapclient;
    ros::ServiceClient moveBaseMapResetClient;
    ros::ServiceClient hsMapReset;

    ros::ServiceClient wallsClient;

    // Transform listenerString
    tf::TransformListener tfl;

    // Action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

    // Subscriber callbacks
    void tcscallback(const std_msgs::Char::ConstPtr &msg);
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
    void localizationCallback(const geometry_msgs::Point::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    void timerCallback(const ros::TimerEvent &event);

    // Action client callbacks
    void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result);
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    void activeCb();

    // Action client functions
    void movementClientAsync(move_base_msgs::MoveBaseGoal goal);
    void sendGoal(geometry_msgs::Pose pose);
    void sendMapGoal(int movement, int rDirection);

    // Return values:
    // 0: Goal not reached, black tile
    // 1: Goal reached, white tile
    // 2: Goal reached, blue tile
    int sendMapGoalGOAT(int movement, int rDirection);

    int checkDownRamp();
    int checkBumperStairs();

    void clearMap();
    void cancelGoal();

    void goNorth();
    void goEast();
    void goSouth();
    void goWest();

    double yawDifference(double yaw1, double yaw2);
#else

    ros::Subscriber jetsonresultsub;
    ros::Publisher unitmovementpub;

    void jetsonResultCallback(const std_msgs::Int8::ConstPtr &msg);

#endif

    ClientScope scope;

    bool upRamp;

    // VLX data
    double distVlxFront;
    double distVlxRight;
    double distVlxLeft;

    bool receivedVlxFront;
    bool receivedVlxRight;
    bool receivedVlxLeft;

    double distLidar;

    ros::Subscriber vlxfrontsub;
    ros::Subscriber vlxrightsub;
    ros::Subscriber vlxleftsub;

    ros::Subscriber startsub;

    // Publishers
    ros::Publisher dispenserpub;
    ros::Publisher lidarserialpub;
    ros::Publisher debugpub;

    ros::ServiceClient victimsClient;
    ros::ServiceClient wallsDistClient;

    ros::ServiceClient vlxDistClient;
    ros::ServiceClient goalStatusClient;
    ros::ServiceClient robotStartClient;

    ros::ServiceClient test_lidar_client;

    int checkUpRamp();

    void vlxFrontCallback(const sensor_msgs::Range::ConstPtr &msg);
    void vlxRightCallback(const sensor_msgs::Range::ConstPtr &msg);
    void vlxLeftCallback(const sensor_msgs::Range::ConstPtr &msg);

    void startCallback(const std_msgs::Int8::ConstPtr &msg);
    int sendGoalJetson(int movement);

    void sendKit();

public:
#ifdef simulateRos
    Map *mapa;
#endif

#ifdef useNavStack
    char tcsdata;

    void publishIdealOrientation(int orientation);
#endif

    bool startAlgorithm;

    ROSbridge(ros::NodeHandle *n);

    int sendUnitGoal(int movement, int rDirection);

    vector<bool> getWalls();
    int getVictims();
    bool checkStart();

    void pubDebug(string msg);
};

#ifdef useNavStack

ROSbridge::ROSbridge(ros::NodeHandle *n) : ac("move_base", true) //, tfb(n)
{
    NOSEUP_PITCH = -0.1;
    NOSEDOWN_PITCH = 0.1;

    debugging = true;
    debugmapgoals = true;
    startedImu = false;
    xdistCenter = 0;
    ydistCenter = 0;
    tcsdata = '0';
    limitSwitchLeft = false;

    blueTile = false;
    blackTile = false;
    silverTile = false;

    receivedVlxFront = false;
    receivedVlxRight = false;
    receivedVlxLeft = false;

    upRamp = false;
    downRamp = false;

    restartGoal = false;

    startAlgorithm = false;

    ROS_INFO("Creating ROSbridge");
    nh = n;

    ROS_INFO("Creating subscribers");

    tcssub = nh->subscribe("/sensor/tcs", 1000, &ROSbridge::tcscallback, this);
    centersub = nh->subscribe("/center_location", 10, &ROSbridge::localizationCallback, this);
    imusub = nh->subscribe("/imu/data", 10, &ROSbridge::imuCallback, this);
    vlxfrontsub = nh->subscribe("/sensor/vlx/front", 10, &ROSbridge::vlxFrontCallback, this);
    vlxrightsub = nh->subscribe("/sensor/vlx/right", 10, &ROSbridge::vlxRightCallback, this);
    vlxleftsub = nh->subscribe("/sensor/vlx/left", 10, &ROSbridge::vlxLeftCallback, this);

    jetsonresultsub = nh->subscribe("/control_feedback", 10, &ROSbridge::jetsonResultCallback, this);
    startsub = nh->subscribe("/robot_init", 10, &ROSbridge::startCallback, this);

    ROS_INFO("Created subscribers");

    dispenserpub = nh->advertise<std_msgs::Int16>("/dispenser", 1000);
    debugpub = nh->advertise<std_msgs::String>("/debug", 1000);
    orientationpub = nh->advertise<std_msgs::Float64>("/ideal_orientation", 1000);
    twistpub = nh->advertise<geometry_msgs::Twist>("/recov_vel", 1000);
    pitchpub = nh->advertise<std_msgs::Float64>("/pitch", 1000);
    yawpub = nh->advertise<std_msgs::Float64>("/yaw", 1000);

    unitmovementpub = nh->advertise<std_msgs::Int8>("/unit_movement", 1000);

    // // Transform broadcaster
    // transformStamped = geometry_msgs::TransformStamped();
    // transformStamped.header.frame_id = "base_link";
    // transformStamped.child_frame_id = "perfect_position";

    // mapclient = nh->serviceClient<nav_msgs::GetMap>();
    moveBaseMapResetClient = nh->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    hsMapReset = nh->serviceClient<std_srvs::Trigger>("/reset_map");

    wallsClient = nh->serviceClient<nav_main::GetWalls>("/get_walls");
    wallsDistClient = nh->serviceClient<nav_main::GetWallsDist>("/get_walls_dist");

    ROS_INFO("Created service clients");

    // tfb.run();
    // ROS_INFO("Started transform broadcaster");

    // tfl.waitForTransform("base_link", "perfect_position", ros::Time(0), ros::Duration(10.0));
}

#else

ROSbridge::ROSbridge(ros::NodeHandle *n)
{
    debugging = true;

    distVlxFront = 0;
    distVlxRight = 0;
    distVlxLeft = 0;
    distLidar = 0;

    receivedVlxFront = false;
    receivedVlxRight = false;
    receivedVlxLeft = false;

    startAlgorithm = true;

    ROS_INFO("Creating ROSbridge");
    nh = n;

    ROS_INFO("Creating subscribers");

    // vlxfrontsub = nh->subscribe("/sensor/vlx/front", 10, &ROSbridge::vlxFrontCallback, this);
    // vlxrightsub = nh->subscribe("/sensor/vlx/right", 10, &ROSbridge::vlxRightCallback, this);
    // vlxleftsub = nh->subscribe("/sensor/vlx/left", 10, &ROSbridge::vlxLeftCallback, this);
    // startsub = nh->subscribe("/robot_init", 10, &ROSbridge::startCallback, this);

    jetsonresultsub = nh->subscribe("/control_feedback", 10, &ROSbridge::jetsonResultCallback, this);

    ROS_INFO("Created subscribers");

    dispenserpub = nh->advertise<std_msgs::Int8>("/dispenser", 1000);
    lidarserialpub = nh->advertise<std_msgs::Float32MultiArray>("/lidar_serial", 1000);
    debugpub = nh->advertise<std_msgs::String>("/debug", 1000);

    unitmovementpub = nh->advertise<std_msgs::Int8>("/unit_movement", 1000);

    victimsClient = nh->serviceClient<openmv_camera::BothCameras>("/get_victims");

    vlxDistClient = nh->serviceClient<exploration::VLXDist>("/get_vlx");
    goalStatusClient = nh->serviceClient<exploration::GoalStatus>("/get_goal_status");
    robotStartClient = nh->serviceClient<std_srvs::Trigger>("/get_start_status");
    wallsDistClient = nh->serviceClient<nav_main::GetWallsDist>("/get_walls_dist");

    test_lidar_client = nh->serviceClient<std_srvs::Trigger>("/get_lidar_status");
}

#endif

bool ROSbridge::checkStart()
{
    // call service and return value
    std_srvs::Trigger srv;
    robotStartClient.waitForExistence();
    ROS_INFO("Waiting for start service");
    robotStartClient.call(srv);

    startAlgorithm = srv.response.success;
    ROS_INFO("Start service returned: %d", startAlgorithm);

    return startAlgorithm;
}

// Subscriber callbacks

#ifdef useNavStack

void ROSbridge::tcscallback(const std_msgs::Char::ConstPtr &msg)
{
    ROS_INFO("I heard: [%c]", msg->data);
    tcsdata = msg->data;
    // tcsdata = 'b';
}

void ROSbridge::statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    ROS_INFO("StatusCallback: %d", msg->status_list[0].status);
    if (scope.startedGoal)
        scope.status = msg->status_list[0].status;
}

void ROSbridge::localizationCallback(const geometry_msgs::Point::ConstPtr &msg)
{
    // ROS_INFO("LocalizationCallback: %f, %f", msg->x, msg->y);
    xdistCenter = msg->x;
    ydistCenter = msg->y;
}

void ROSbridge::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    xImu = msg->orientation.x;
    yImu = msg->orientation.y;
    zImu = msg->orientation.z;
    wImu = msg->orientation.w;

    // Roll, pitch, yaw
    tf::Quaternion q(xImu, yImu, zImu, wImu);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    if (!startedImu)
    {
        startedImu = true;
        northYaw = yaw;

        // Range: -pi to pi
        // Clockwise gives smaller values
        // Counter-clockwise gives larger values

        ROS_INFO("northYaw: %f", northYaw);

        if (northYaw > -M_PI_2)
            eastYaw = northYaw - M_PI_2;
        else
            eastYaw = northYaw + M_PI_2 * 3;

        // South
        if (northYaw > 0)
            southYaw = northYaw - M_PI;
        else
            southYaw = northYaw + M_PI;

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

    if (false)
    {
        // ROS_INFO("x: %f", xImu);
        // ROS_INFO("y: %f", yImu);
        // ROS_INFO("z: %f", zImu);
        // ROS_INFO("w: %f", wImu);
        // ROS_INFO("roll: %f", roll);
        // ROS_INFO("pitch: %f", pitch);
        ROS_INFO("yaw: %f", yaw);
        ROS_INFO("Pitch: %f", pitch);
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

// Timer callback
void ROSbridge::timerCallback(const ros::TimerEvent &)
{
    // ROS_INFO("Timer callback");
    // ROS_INFO("Timer callback: %d", scope.status);

    // Check if goal is reached
    if (!scope.resultReceived)
    {
        if (!scope.resultReceived)
        {
            // Cancel and restart goal
            restartGoal = true;

            ac.cancelGoal();
        }
    }
    // if (scope.status == 3)
    // {
    //     ROS_INFO("Goal reached");
    //     scope.status = 0;
    //     scope.resultReceived = true;
    //     scope.startedGoal = false;
    // }
}

void ROSbridge::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    ros::spinOnce();

    // ROS_INFO("Feedback x %f", feedback->base_position.pose.position.x);
    // ROS_INFO("Feedback y %f", feedback->base_position.pose.position.y);
    // ROS_INFO("Feedback z %f", feedback->base_position.pose.position.z);

    // check color
    if (tcsdata == 'n' && !blackTile)
    {
        if (debugging)
            ROS_INFO("Black tile, Cancelling goal");

        blackTile = true;
        ac.cancelGoal();
    }
    else if (tcsdata == 'a')
    {
        if (debugging)
            ROS_INFO("Blue tile detected");

        blueTile = true;
    }
    else if (tcsdata == 's')
    {
        if (debugging)
            ROS_INFO("Silver tile detected");

        silverTile = true;
    }

    // check limit switches
    if (limitSwitchLeft && limitSwitchRight)
    {
        // Go back
        geometry_msgs::Twist moveBackTwist;
        moveBackTwist.linear.x = -0.2; // move backward at 0.5 m/s
        moveBackTwist.angular.z = 0.0; // turn right at 45 degrees/s (0.785 radians/s)

        // Publish twist message
        twistpub.publish(moveBackTwist);

        // Wait for 1 second
        ros::Duration(0.5).sleep();

        // Stop the robot
        moveBackTwist.linear.x = 0.0;
        moveBackTwist.angular.z = 0.0;
        twistpub.publish(moveBackTwist);
    }
    if (limitSwitchLeft)
    {
        ROS_INFO("Limit switch 1, recovering");

        geometry_msgs::Twist moveBackTwist;
        moveBackTwist.linear.x = -0.2;  // move backward at 0.5 m/s
        moveBackTwist.angular.z = -0.5; // turn right at 45 degrees/s (0.785 radians/s)

        // Publish twist message
        twistpub.publish(moveBackTwist);

        // Wait for 1 second
        ros::Duration(0.5).sleep();

        // Stop the robot
        moveBackTwist.linear.x = 0.0;
        moveBackTwist.angular.z = 0.0;
        twistpub.publish(moveBackTwist);
    }
    if (limitSwitchRight)
    {
        ROS_INFO("Limit switch right, recovering");

        geometry_msgs::Twist moveBackTwist;
        moveBackTwist.linear.x = -0.2; // move backward at 0.5 m/s
        moveBackTwist.angular.z = 0.5; // turn right at 45 degrees/s (0.785 radians/s)

        // Publish twist message
        twistpub.publish(moveBackTwist);

        // Wait for 1 second
        ros::Duration(0.5).sleep();

        // Stop the robot
        moveBackTwist.linear.x = 0.0;
        moveBackTwist.angular.z = 0.0;
        twistpub.publish(moveBackTwist);
    }

    // check pitch from imu

    if (!upRamp && pitch < NOSEUP_PITCH) // Found obstacle (bumper / stairs)
    {
        if (debugging)
            ROS_INFO("Found obstacle");

        obstacle = true;
    }
    else if (pitch > NOSEDOWN_PITCH) // Down ramp
    {
        if (debugging)
            ROS_INFO("Down ramp");

        downRamp = true;

        ac.cancelGoal();
    }
    // check if there's ramp and imu is close to 0
    else if (upRamp && pitch > NOSEUP_PITCH && pitch < NOSEDOWN_PITCH)
    {
        if (debugging)
            ROS_INFO("Up ramp");

        upRamp = true;
    }

    if (upRamp || downRamp)
    {
        // publish forward twist while pitch is not close to 0
        geometry_msgs::Twist moveTwist;
        moveTwist.linear.x = 0.2; // move forward at 0.5 m/s
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

    // Start timer for 7 seconds to check if goal is reached
    timer = nh->createTimer(ros::Duration(7), &ROSbridge::timerCallback, this, true);
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
    poseStamped.header.frame_id = "perfect_position";
    poseStamped.pose = pose;

    // tfl.waitForTransform("base_link", "perfect_position", ros::Time(0), ros::Duration(1.0));
    // tfl.transformPose("base_link", ros::Time(0), poseStamped, "perfect_position", goal.target_pose);
    // // tfl.transformPose("base_link", poseStamped, goal.target_pose);

    // tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
    // tfl.transformPose("map", ros::Time(0), goal.target_pose, "base_link", goal.target_pose);
    // tfl.transformPose("map", goal.target_pose, goal.target_pose);

    // goal.target_pose = poseStamped;
    movementClientAsync(goal);
}

#else

void ROSbridge::jetsonResultCallback(const std_msgs::Int8::ConstPtr &msg)
{
    ROS_INFO("Jetson result: %d", msg->data);
    scope.result = msg->data;
    scope.resultReceived = true;
    scope.startedGoal = false;
}

#endif

int ROSbridge::checkUpRamp()
{
    ros::spinOnce();

    // Call vlx service
    exploration::VLXDist vlxDistSrv;
    vlxDistClient.waitForExistence();
    vlxDistClient.call(vlxDistSrv);

    distVlxFront = vlxDistSrv.response.front ;
    distVlxRight = vlxDistSrv.response.right;
    distVlxLeft = vlxDistSrv.response.left;

    ROS_INFO("Got vlx walls: front:%f, right: %f, left: %f", distVlxFront, distVlxRight, distVlxLeft);

    // Check for up ramps in front, if the front distance of the laser scan and the vlx are less than 30 cm and the difference between the two is between 5 and 10 cm, then it is an up ramp
    if (distLidar < 0.4 && (distLidar + 7) - distVlxFront < 0.2 && (distLidar + 7) - distVlxFront > 0.1)
    {
        ROS_INFO("Up ramp");
        // upRamp = true;

        return 1;
    }

    return 0;
}

void ROSbridge::vlxFrontCallback(const sensor_msgs::Range::ConstPtr &msg)
{
    receivedVlxFront = true;
    distVlxFront = msg->range;
}

void ROSbridge::vlxRightCallback(const sensor_msgs::Range::ConstPtr &msg)
{
    receivedVlxRight = true;
    distVlxRight = msg->range;
}

void ROSbridge::vlxLeftCallback(const sensor_msgs::Range::ConstPtr &msg)
{
    receivedVlxLeft = true;
    distVlxLeft = msg->range;
}

void ROSbridge::startCallback(const std_msgs::Int8::ConstPtr &msg)
{
    ROS_INFO("Start callback: %d", msg->data);
    startAlgorithm = msg->data;
}

// 0: forward
// 1: left
// 3: right

// Return values:
// 0: Goal not reached, black tile
// 1: Goal reached, white tile
// 2: Goal reached, blue tile / obstacle
// 3: Goal reached, silver tile
// 4: Goal reached, down ramp
// 5: Goal reached, up ramp
int ROSbridge::sendGoalJetson(int movement)
{
    ROS_INFO("Send goal to arduino");

    ros::spinOnce();

    // Call get_walls_dist service
    nav_main::GetWallsDist walls;

    wallsDistClient.waitForExistence();
    wallsDistClient.call(walls);

    if (debugging)
    {
        ROS_INFO("Got walls: front:%f, right: %f, back: %f, left: %f", walls.response.front, walls.response.right, walls.response.back, walls.response.left);
    }

    distLidar = walls.response.front;

    std_msgs::Int8 movementmsg;
    if (movement == 0)
    {
        // Check if there's a ramp
        if (checkUpRamp())
        {
            ROS_INFO("Forward (ramp)");
            upRamp = true;
            movementmsg.data = 4;
        }
        else
        {
            ROS_INFO("Forward");
            movementmsg.data = 0;
        }
    }
    else if (movement == 1)
    {
        ROS_INFO("Turn right");
        movementmsg.data = 1;
    }
    else if (movement == 2)
    {
        ROS_INFO("Backward");
        movementmsg.data = 2;
    }
    if (movement == 3)
    {
        ROS_INFO("Turn left");
        movementmsg.data = 3;
    }
    // if (movement == 4)
    // {
    //     ROS_INFO("Forward (ramp)");
    //     movementmsg.data = 4;
    // }
    if (movement == 4)
    {
        // ROS_INFO("Backward");
        // movementmsg.data = 2;
    }

    unitmovementpub.publish(movementmsg);
    scope.startedGoal = true;
    scope.resultReceived = false;
    scope.status = -1;

    ROS_INFO("Status %d: ", scope.status);

    // wait for message from jetson (jetsonresultsub)
    do
    {
        ROS_INFO("Waiting for result");

        ros::spinOnce();

        // Call get_walls_dist service
        nav_main::GetWallsDist walls;
        wallsDistClient.waitForExistence();
        wallsDistClient.call(walls);
        float distLidarFront = walls.response.front;
        float distLidarRight = walls.response.right;
        float distLidarLeft = walls.response.left;
        float distLidarBack = walls.response.back;

        // Make float multiarray msg for lidar
        std_msgs::Float32MultiArray lidarserialmsg;
        lidarserialmsg.data.clear();
        lidarserialmsg.data.push_back(distLidarFront);
        lidarserialmsg.data.push_back(distLidarRight);
        lidarserialmsg.data.push_back(distLidarLeft);
        lidarserialmsg.data.push_back(distLidarBack);

        lidarserialpub.publish(lidarserialmsg);

        // call status service
        exploration::GoalStatus statusSrv;
        goalStatusClient.waitForExistence();
        goalStatusClient.call(statusSrv);

        scope.status = statusSrv.response.status;

        ROS_INFO("Status %d: ", scope.status);

        // // check color
        // if (tcsdata == 'N' && !blackTile)
        // {
        //     if (debugging)
        //         ROS_INFO("Black tile, Cancelling goal");

        //     blackTile = true;
        //     // ac.cancelGoal();
        // }
        // else if (tcsdata == 'A')
        // {
        //     if (debugging)
        //         ROS_INFO("Blue tile detected");

        //     blueTile = true;
        // }
        // else if (tcsdata == 'S' || tcsdata == 'M')
        // {
        //     if (debugging)
        //         ROS_INFO("Silver tile detected");

        //     silverTile = true;
        // }

        // // check pitch from imu

        // if (!upRamp && pitch < NOSEUP_PITCH) // Found obstacle (bumper / stairs)
        // {
        //     if (debugging)
        //         ROS_INFO("Found obstacle");

        //     obstacle = true;
        // }
        // else if (pitch > NOSEDOWN_PITCH) // Down ramp
        // {
        //     if (debugging)
        //         ROS_INFO("Down ramp");

        //     downRamp = true;

        //     // ac.cancelGoal();
        // }
        // // check if there's ramp and imu is close to 0
        // else if (upRamp && pitch > NOSEUP_PITCH && pitch < NOSEDOWN_PITCH)
        // {
        //     if (debugging)
        //         ROS_INFO("Up ramp");

        //     upRamp = true;
        // }
    } while (scope.status == -1);

    scope.result = scope.status;

    ROS_INFO("Received status: %d", scope.status);

    // Test lidar status
    if (true)
    {
        std_srvs::Trigger lidarSrv;
        test_lidar_client.waitForExistence();
        test_lidar_client.call(lidarSrv);

        ROS_INFO("Lidar status: %d", lidarSrv.response.success);
    }

    receivedVlxFront = false;
    receivedVlxRight = false;
    receivedVlxLeft = false;

    if (upRamp)
    {
        ROS_INFO("Returning 5 (up ramp)");
        upRamp = false;
        return 5;
    }

    if (scope.result == 4)
    {
        // turn right and check if there's a ramp
        sendGoalJetson(1);
        if (checkUpRamp())
        {
            ROS_INFO("Turning and Returning 4 (down ramp)");
            // if there was a ramp, turn left and return the value
            sendGoalJetson(3);
            return 4;
        }
        else
        {
            ROS_INFO("Turning and Returning 2 (stairs)");
            // if there was no ramp, turn left and return the value
            sendGoalJetson(3);
            return 0;
        }
    }

    ROS_INFO("Returning %d", scope.result);
    return scope.result;

    // if (blackTile)
    // {
    //     blackTile = false;
    //     return 0;
    // }
    // else if (blueTile)
    // {
    //     ros::Duration(5).sleep();
    //     blueTile = false;
    //     return 2;
    // }
    // else if (silverTile)
    // {
    //     silverTile = false;
    //     return 3;
    // }
    // else if (obstacle)
    // {
    //     obstacle = false;
    //     return 2;
    // }
    // else if (downRamp)
    // {
    //     downRamp = false;
    //     return 4;
    // }
    // else if (upRamp)
    // {
    //     upRamp = false;
    //     return 5;
    // }
    // else
    // {
    //     return 1;
    // }

    // return 0;
}

#ifdef useNavStack
// Return values:
// 0: Goal not reached, black tile
// 1: Goal reached, white tile
// 2: Goal reached, blue tile / obstacle
// 3: Goal reached, silver tile
// 4: Goal reached, down ramp
// 5: Goal reached, up ramp
int ROSbridge::sendMapGoalGOAT(int movement, int rDirection)
{
    // Call get_walls_dist service
    nav_main::GetWallsDist walls;

    wallsClient.call(walls);

    // Get the distance from the lidar
    distLidar = walls.response.front;

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "perfect_position";
    if (movement == 0)
    {
        ROS_INFO("Forward");

        pose.pose.position.x = 0.3;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        // Check for up ramps in front, if the front distance of the laser scan and the vlx are less than 30 cm and the difference between the two is between 5 and 10 cm, then it is an up ramp
        if (distLidar < 0.3 && distLidar - distVlxFront < 0.1 && distLidar - distVlxFront > 0.05)
        {
            ROS_INFO("Up ramp");
            // upRamp = true;

            // Publish twist message
            geometry_msgs::Twist moveTwist;
            moveTwist.linear.x = 0.2; // move forward at 0.5 m/s

            while (pitch > 0.05 || pitch < -0.05)
            {
                twistpub.publish(moveTwist);

                ros::spinOnce();
            }

            // Stop publishing twist message
            moveTwist.linear.x = 0;
            twistpub.publish(moveTwist);

            return 5;
        }
    }
    if (movement == 1) // Turn right
    {
        ROS_INFO("Turn right");

        pose.pose.position.x = 0.001;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI / 2);
    }
    if (movement == 3) // Turn left
    {
        ROS_INFO("Turn left");
        pose.pose.position.x = 0.001;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);
    }

    // Transform pose to map frame
    geometry_msgs::PoseStamped poseMap;
    tfl.waitForTransform("map", "perfect_position", ros::Time(0), ros::Duration(1.0));
    tfl.transformPose("map", ros::Time(0), pose, "perfect_position", poseMap);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = poseMap;

    if (blueTile) // Wait 5 seconds if blue tile is detected
    {
        ROS_INFO("Blue tile detected, waiting 5 seconds");
        ros::Duration(5).sleep();
        blueTile = false;
    }
    else if (silverTile)
    {
        silverTile = false;
    }

    movementClientAsync(goal);

    // Return value if goal is reached
    if (blackTile)
    {
        // Send goal to 0,0 to go to the center of the tile
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "perfect_position";
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        // Transform pose to map frame
        geometry_msgs::PoseStamped poseMap;
        tfl.waitForTransform("map", "perfect_position", ros::Time(0), ros::Duration(1.0));
        tfl.transformPose("map", ros::Time(0), pose, "perfect_position", poseMap);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = poseMap;

        movementClientAsync(goal);

        blackTile = false;
        return 0;
    }
    else if (blueTile)
    {
        return 2;
    }
    else if (obstacle)
    {
        obstacle = false;
        return 2;
    }
    else if (silverTile)
    {
        return 3;
    }
    else if (restartGoal)
    {
        restartGoal = false;

        // Restart maps
        clearMap();

        // Send goal to 0,0 to go to the center of the tile
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "perfect_position";
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        // Transform pose to map frame
        geometry_msgs::PoseStamped poseMap;
        tfl.waitForTransform("map", "perfect_position", ros::Time(0), ros::Duration(1.0));
        tfl.transformPose("map", ros::Time(0), pose, "perfect_position", poseMap);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose = poseMap;

        movementClientAsync(goal);

        return sendMapGoalGOAT(movement, rDirection);
    }
    else if (downRamp)
    {
        geometry_msgs::Twist moveTwist;
        moveTwist.linear.x = -0.2; // move forward at 0.5 m/s

        while (pitch > 0.1 || pitch < -0.1)
        {
            ros::spinOnce();

            twistpub.publish(moveTwist);
        }

        // Stop publishing twist message
        moveTwist.linear.x = 0;
        twistpub.publish(moveTwist);

        downRamp = false;

        // Reset maps
        clearMap();

        return 4;
    }
    else if (upRamp)
    {
        clearMap();

        upRamp = false;
        return 5;
    }
    else
    {
        return 1;
    }
}

#endif

// Return values:
// 0: Goal not reached, black tile
// 1: Goal reached, white tile
// 2: Goal reached, blue tile / obstacle
// 3: Goal reached, silver tile
// 4: Goal reached, down ramp
// 5: Goal reached, up ramp
int ROSbridge::sendUnitGoal(int movement, int rDirection)
{
#ifdef simulateRos

    if (movement != 0)
    {
        return 1;
    }

    map<int, string> invDirections = {
        {0, "north"},
        {1, "east"},
        {2, "south"},
        {3, "west"}};

    char c = mapa->getChar(invDirections[rDirection]);

    if (c == 'a') // casilla azul
    {
        return 2;
    }
    else if (c == 'n')
        return 0;
    else if (c == 'b') // bumper
    {
        return 2;
    }
    else if (c == '/') // escaleras
    {
        return 2;
    }
    else if (c == 'r') // rampa
    {
        return (mapa->rampDirection(invDirections[rDirection]) == rDirection) ? 5 : 4;
    }

    return 1;
#else

#ifndef useNavStack
    return sendGoalJetson(movement);
#else

    if (false)
    {
        return sendMapGoalGOAT(movement, rDirection);
    }

    if (sendMapGoals)
    {
        sendMapGoal(movement, rDirection);

        return 0;
    }

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

        // switch (rDirection)
        // {
        // case 0: // North
        //     tfb.setTransformYaw(yawDifference(northYaw, yaw));
        //     break;
        // case 1: // East
        //     tfb.setTransformYaw(yawDifference(eastYaw, yaw));
        //     break;
        // case 2: // South
        //     tfb.setTransformYaw(yawDifference(southYaw, yaw));
        //     break;
        // case 3: // West
        //     tfb.setTransformYaw(yawDifference(westYaw, yaw));
        //     break;
        // default:
        //     tfb.setTransformYaw(yawDifference(northYaw, yaw));
        //     break;
        // }

        if (rDirection == 0) // North
        {
            if (ydistCenter != -31)
                pose.position.x += ydistCenter * 0.01;
            if (xdistCenter != -31)
                pose.position.y = xdistCenter * -0.01;
        }
        else if (rDirection == 1) // East
        {
            if (xdistCenter != -31)
                pose.position.x += xdistCenter * 0.01;
            if (ydistCenter != -31)
                pose.position.y = ydistCenter * 0.01;
        }
        else if (rDirection == 2) // South
        {
            if (ydistCenter != -31)
                pose.position.x += ydistCenter * -0.01;
            if (xdistCenter != -31)
                pose.position.y = xdistCenter * 0.01;
        }
        else if (rDirection == 3) // West
        {
            if (xdistCenter != -31)
                pose.position.x += xdistCenter * -0.01;
            if (ydistCenter != -31)
                pose.position.y = ydistCenter * -0.01;
        }
        pubDebug("Pose x: " + std::to_string(pose.position.x) + " y: " + std::to_string(pose.position.y));
    }
    else if (movement == 1) // Turn right
    {
        ROS_INFO("Turn right");

        pose.position.x = 0.001;
        pose.position.y = 0;
        pose.position.z = 0;
        // pose.orientation.x = 0;
        // pose.orientation.y = 0;
        // pose.orientation.z = 0;
        // pose.orientation.w = 1;

        // Calculate angle to get 90 degree turn
        // double yawAngle;

        // if (rDirection == 0) // North
        // {
        //     yawAngle = eastYaw - yaw;
        // }
        // else if (rDirection == 1) // East
        // {
        //     yawAngle = southYaw - yaw;
        // }
        // else if (rDirection == 2) // South
        // {
        //     yawAngle = westYaw - yaw;
        // }
        // else if (rDirection == 3) // West
        // {
        //     yawAngle = northYaw - yaw;
        // }

        // switch (rDirection)
        // {
        // case 0: // North
        //     tfb.setTransformYaw(yawDifference(eastYaw, yaw));
        //     break;
        // case 1: // East
        //     tfb.setTransformYaw(yawDifference(southYaw, yaw));
        //     break;
        // case 2: // South
        //     tfb.setTransformYaw(yawDifference(westYaw, yaw));
        //     break;
        // case 3: // West
        //     tfb.setTransformYaw(yawDifference(northYaw, yaw));
        //     break;

        // default:
        //     break;
        // }

        pose.orientation = tf::createQuaternionMsgFromYaw(0);

        // pose.pose.position.x = 0;
        // pose.pose.position.y = 0.3;
        // pose.pose.position.z = 0;
        // pose.pose.orientation.x = 0;
        // pose.pose.orientation.y = 0;
        // pose.pose.orientation.z = 0;
        // pose.pose.orientation.w = 1;
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

        switch (rDirection)
        {
        case 0: // North
            transformYaw = southYaw;
            break;
        case 1: // East
            transformYaw = westYaw;
            break;
        case 2: // South
            transformYaw = northYaw;
            break;
        case 3: // West
            transformYaw = eastYaw;
            break;
        }
    }
    else if (movement == 3) // Turn left
    {
        ROS_INFO("Turn left");

        pose.position.x = 0.001;
        pose.position.y = 0;
        pose.position.z = 0;
        // pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);

        // switch (rDirection)
        // {
        // case 0: // North
        //     tfb.setTransformYaw(yawDifference(westYaw, yaw));
        //     break;
        // case 1: // East
        //     tfb.setTransformYaw(yawDifference(northYaw, yaw));
        //     break;
        // case 2: // South
        //     tfb.setTransformYaw(yawDifference(eastYaw, yaw));
        //     break;
        // case 3: // West
        //     tfb.setTransformYaw(yawDifference(southYaw, yaw));
        //     break;
        // }

        // // Calculate angle to get 90 degree turn
        double yawAngle;

        if (rDirection == 0) // North
        {
            yawAngle = westYaw;
        }
        else if (rDirection == 1) // East
        {
            yawAngle = northYaw;
        }
        else if (rDirection == 2) // South
        {
            yawAngle = eastYaw;
        }
        else if (rDirection == 3) // West
        {
            yawAngle = southYaw;
        }

        //    */
        pose.orientation = tf::createQuaternionMsgFromYaw(0);
    }

    else if (movement == 4) // Turn around
    {
        // ROS_INFO("Turn around");

        pose.position.x = 0;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation = tf::createQuaternionMsgFromYaw(M_PI * 4);
    }

    // publishTransform();
    // tfl.waitForTransform("perfect_position", "base_link", ros::Time(0), ros::Duration(1.0));
    // tfl.transformPose("perfect_position", pose, pose);

    sendGoal(pose);

    return 0;
#endif
#endif
}

#ifdef useNavStack
void ROSbridge::sendMapGoal(int movement, int rDirection)
{
    if (debugmapgoals)
    {
        ROS_INFO(" ");
        ROS_INFO("Send map goal");
    }
    try
    {
        tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tfl.lookupTransform("map", "base_link", ros::Time(0), baselink_mapTransform);

        ROS_INFO("Transform looked up, x: %f y: %f", baselink_mapTransform.getOrigin().x(), baselink_mapTransform.getOrigin().y());
    }
    catch (const tf::TransformException &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR("Transform error: %s", e.what());
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = baselink_mapTransform.getOrigin().x() + xdistCenter * 0.01;
    goal.target_pose.pose.position.y = baselink_mapTransform.getOrigin().y() + ydistCenter * 0.01;
    goal.target_pose.pose.position.z = baselink_mapTransform.getOrigin().z();

    // goal.target_pose.pose.orientation.x = baselink_mapTransform.getRotation().x();
    // goal.target_pose.pose.orientation.y = baselink_mapTransform.getRotation().y();
    // goal.target_pose.pose.orientation.z = baselink_mapTransform.getRotation().z();
    // goal.target_pose.pose.orientation.w = baselink_mapTransform.getRotation().w();

    if (movement == 0)
    {
        // goal.target_pose.pose.position.x += 0.3;

        switch (rDirection)
        {
        case 0:
            goal.target_pose.pose.position.x += 0.3;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yawDifference(northYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: North, yaw Difference: %f", yawDifference(northYaw, yaw));
            }
            break;

        case 1:
            goal.target_pose.pose.position.y -= 0.3;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yawDifference(eastYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: East, yaw Difference: %f", yawDifference(eastYaw, yaw));
            }
            break;

        case 2:
            goal.target_pose.pose.position.x -= 0.3;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yawDifference(southYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: South, yaw Difference: %f", yawDifference(southYaw, yaw));
            }
            break;

        case 3:
            goal.target_pose.pose.position.y += 0.3;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yawDifference(westYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: West, yaw Difference: %f", yawDifference(westYaw, yaw));
            }
            break;

        default:
            break;
        }
    }
    else if (movement == 1) // Turn right
    {
        tf::Matrix3x3 m(baselink_mapTransform.getRotation());
        double originYaw, pitch, roll;
        m.getRPY(roll, pitch, originYaw);

        switch (rDirection)
        {
        case 0:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI_2 + yawDifference(northYaw, originYaw));

            if (debugmapgoals)
            {
                ROS_INFO("East yaw: %f, origin yaw: %f", eastYaw, originYaw);
                ROS_INFO("Objective: East, yaw Difference: %f", -M_PI_2 + yawDifference(northYaw, originYaw));
            }
            break;

        case 1:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI_2 + yawDifference(eastYaw, originYaw));

            if (debugmapgoals)
            {
                ROS_INFO("South yaw: %f, origin yaw: %f", southYaw, originYaw);
                ROS_INFO("Objective: South, yaw Difference: %f", -M_PI_2 + yawDifference(southYaw, yaw));
            }
            break;

        case 2:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI_2 + yawDifference(southYaw, originYaw));

            if (debugmapgoals)
            {
                ROS_INFO("West yaw: %f, origin yaw: %f", westYaw, originYaw);
                ROS_INFO("Objective: West, yaw Difference: %f", -M_PI_2 + yawDifference(westYaw, originYaw));
            }
            break;

        case 3:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-M_PI_2 + yawDifference(westYaw, originYaw));

            if (debugmapgoals)
            {
                ROS_INFO("North yaw: %f, origin yaw: %f", northYaw, originYaw);
                ROS_INFO("Objective: North, yaw Difference: %f", -M_PI_2 + yawDifference(northYaw, originYaw));
            }
            break;

        default:
            break;
        }
    }
    // else if (movement == 2)
    // {
    //     goal.target_pose.pose.position.x -= 0.3;
    // }
    else if (movement == 3) // Turn left
    {
        switch (rDirection)
        {
        case 0:

            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI_2 + yawDifference(westYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: West, yaw Difference: %f", M_PI_2 + yawDifference(westYaw, yaw));
                ROS_INFO("Yaw goal: %f", M_PI_2 + yawDifference(westYaw, yaw));
            }
            break;

        case 1:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI_2 + yawDifference(northYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: North, yaw Difference: %f", M_PI_2 + yawDifference(northYaw, yaw));
                ROS_INFO("Yaw goal: %f", M_PI_2 + yawDifference(northYaw, yaw));
            }

            break;

        case 2:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI_2 + yawDifference(eastYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: East, yaw Difference: %f", M_PI_2 + yawDifference(eastYaw, yaw));
                ROS_INFO("Yaw goal: %f", M_PI_2 + yawDifference(eastYaw, yaw));
            }

            break;

        case 3:
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI_2 + yawDifference(southYaw, yaw));

            if (debugmapgoals)
            {
                ROS_INFO("Objective: South, yaw Difference: %f", M_PI_2 + yawDifference(southYaw, yaw));
                ROS_INFO("Yaw goal: %f", M_PI_2 + yawDifference(southYaw, yaw));
            }

            break;

        default:
            break;
        }
    }

    ROS_INFO("Sending map goal");

    movementClientAsync(goal);
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

#endif

#ifdef simulateRos

vector<bool> ROSbridge::getWalls()
{
    ROS_INFO("Getting walls");

    // {front, right, back, left}
    vector<bool> wallsVector = {0, 0, 0, 0};

    wallsVector[0] = (mapa->getChar("north") == '#') ? 1 : 0;
    wallsVector[1] = (mapa->getChar("east") == '#') ? 1 : 0;
    wallsVector[2] = (mapa->getChar("south") == '#') ? 1 : 0;
    wallsVector[3] = (mapa->getChar("west") == '#') ? 1 : 0;

    return wallsVector;
}

#else

#ifndef useLidar

vector<bool> ROSbridge::getWalls()
{
    ros::spinOnce();
    ROS_INFO("Getting walls");

    // {front, right, back, left}
    vector<bool> wallsVector = {distVlxFront<0.15, distVlxRight> 0.15, 0, distVlxLeft > 0.15};

    // Turn right to check back wall
    sendGoalJetson(1);

    ros::spinOnce();
    wallsVector[2] = distVlxRight > 0.15;

    // Turn left
    sendGoalJetson(3);

    return wallsVector;
}

#else

vector<bool> ROSbridge::getWalls()
{
    // while (!receivedVlxLeft || !receivedVlxRight)
    //     ros::spinOnce();

    ROS_INFO("Getting walls");

    nav_main::GetWallsDist walls;
    wallsDistClient.waitForExistence();
    wallsDistClient.call(walls);

    // call vlx service
    exploration::VLXDist vlx;
    vlxDistClient.waitForExistence();
    vlxDistClient.call(vlx);

    distVlxFront = walls.response.front;
    distVlxRight = vlx.response.right;
    distVlxLeft = vlx.response.left;

    ROS_INFO("Got walls: front:%f, right: %f, back: %f, left: %f", walls.response.front, distVlxRight, walls.response.back, distVlxLeft);

    vector<bool> wallsVector = {walls.response.front < 0.25, distVlxRight < 0.25, walls.response.back < 0.25, distVlxLeft < 0.25};

    return wallsVector;
}

#endif

#endif

void ROSbridge::sendKit()
{
    ROS_INFO("Sending kit");

    std_msgs::String msg;
    msg.data = "1";
    dispenserpub.publish(msg);
}

/* void ROSbridge::publishTransform()
{
ros::spinOnce();
transformStamped.header.stamp = ros::Time::now();
// if (xdistCenter != 31)
// {
//     transformStamped.transform.translation.x = xdistCenter * 0.01;
// }
// else
// {
//     transformStamped.transform.translation.x = 0;
// }
// if (ydistCenter != 31)
// {
//     transformStamped.transform.translation.y = ydistCenter * 0.01;
// }
// else
// {
//     transformStamped.transform.translation.y = 0;
// }
// transformStamped.transform.translation.z = 0.0;

transformStamped.transform.translation.x = 0;
transformStamped.transform.translation.y = 0;
transformStamped.transform.translation.z = 0.0;

ROS_INFO("Yaw: %f, transformYaw: %f", yaw, transformYaw);

if (true)
{
    transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(transformYaw - yaw);
}
// else
// {
//     transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(yaw - transformYaw);
// }

tfbr.sendTransform(transformStamped);
} */

#ifdef simulateRos

int ROSbridge::getVictims()
{
    // Get victims from current position in map
    if (mapa->getCurrentChar() == 'H' || mapa->getCurrentChar() == 'S')
    {
        return 1;
    }

    return 0;
}

#else

int ROSbridge::getVictims()
{
    return 0;
    // Normal tile, check victim
    openmv_camera::BothCameras bothCameras;

    bool gotVictims = false;

    victimsClient.waitForExistence();
    victimsClient.call(bothCameras);

    if (bothCameras.response.left_cam == "H")
    {
        // Drop three kits to the left
        std_msgs::Int8 msg;
        msg.data = -3;
        dispenserpub.publish(msg);

        gotVictims = true;

        // ros::Duration(6).sleep();
    }
    else if (bothCameras.response.right_cam == "H")
    {
        // Drop three kits to the right
        std_msgs::Int8 msg;
        msg.data = 3;
        dispenserpub.publish(msg);

        gotVictims = true;

        // ros::Duration(6).sleep();
    }
    else if (bothCameras.response.left_cam == "r" || bothCameras.response.left_cam == "S")
    {
        // Drop two kits to the left
        std_msgs::Int8 msg;
        msg.data = -2;
        dispenserpub.publish(msg);

        gotVictims = true;

        // ros::Duration(4).sleep();
    }
    else if (bothCameras.response.right_cam == "S")
    {
        // Drop two kits to the right
        std_msgs::Int8 msg;
        msg.data = 2;
        dispenserpub.publish(msg);

        gotVictims = true;

        // ros::Duration(4).sleep();
    }
    else if (bothCameras.response.left_cam == "y" || bothCameras.response.left_cam == "r")
    {
        // Drop one kit to the left
        std_msgs::Int8 msg;
        msg.data = -1;
        dispenserpub.publish(msg);

        gotVictims = true;

        // ros::Duration(2).sleep();
    }
    else if (bothCameras.response.right_cam == "y" || bothCameras.response.right_cam == "r")
    {
        // Drop one kit to the right
        std_msgs::Int8 msg;
        msg.data = 1;
        dispenserpub.publish(msg);

        gotVictims = true;

        // ros::Duration(2).sleep();
    }

    if (gotVictims)
    {
        // Wait for the dispenser to finish

        while (scope.result != 6)
        {
            ros::spinOnce();
        }

        return 1;
    }

    return 0;
}

#endif

void ROSbridge::pubDebug(string m)
{
    if (debugging)
    {
        cout << m << endl;
        // std_msgs::String msg;
        // msg.data = m;
        // debugpub.publish(msg);
    }
}

#ifdef useNavStack

double ROSbridge::yawDifference(double targetYaw, double originYaw)
{
    // return fmod(yaw1 - yaw2 + M_PI, 2 * M_PI) - M_PI;
    // return fmod(targetYaw - originYaw + M_PI, 2 * M_PI) - M_PI;

    // return yaw1 - yaw2;
    // return targetYaw - originYaw;

    double diff = targetYaw - originYaw;
    // return atan2(sin(diff), cos(diff));

    // if (diff > M_PI)
    // {
    //     diff -= 2 * M_PI;
    // }
    // else if (diff < -M_PI)
    // {
    //     diff += 2 * M_PI;
    // }
    return diff;
}

#endif

#ifdef useNavStack

#ifdef simulateRos

void ROSbridge::publishIdealOrientation(int orientation)
{
}

#else

void ROSbridge::publishIdealOrientation(int orientation)
{
    double radians = 0;
    /* switch (orientation)
    {
        case 0: // north
            radians = northYaw;
            break;

        case 1: // east
            radians = eastYaw;
            break;

        case 2: // south
            radians = southYaw;
            break;

        case 3: // west
            radians = westYaw;
            break;

        default:
            break;
    } */
    switch (orientation)
    {
    case 0: // north
        radians = 0;
        break;
    case 1: // east
        radians = -M_PI / 2;
        break;
    case 2: // south
        radians = M_PI;
        break;
    case 3: // west
        radians = M_PI / 2;
        break;
    }
    std_msgs::Float64 msg;
    msg.data = radians;
    orientationpub.publish(msg);
    // Log Ideal Orientation
    ROS_INFO("Ideal Orientation: %f", radians);
}

#endif

#endif

#ifdef useNavStack

void ROSbridge::goNorth()
{
    ROS_INFO("Going north");

    try
    {
        tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tfl.lookupTransform("map", "base_link", ros::Time(0), baselink_mapTransform);
    }
    catch (const tf::TransformException &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR("Transform error: %s", e.what());
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = baselink_mapTransform.getOrigin().x() + 0.3 + ydistCenter;
    goal.target_pose.pose.position.y = baselink_mapTransform.getOrigin().y() + xdistCenter;
    goal.target_pose.pose.position.z = baselink_mapTransform.getOrigin().z();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(northYaw);

    movementClientAsync(goal);
}

void ROSbridge::goEast()
{
    ROS_INFO("Going east");

    try
    {
        tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tfl.lookupTransform("map", "base_link", ros::Time(0), baselink_mapTransform);
    }
    catch (const tf::TransformException &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR("Transform error: %s", e.what());
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = baselink_mapTransform.getOrigin().x() + ydistCenter;
    goal.target_pose.pose.position.y = baselink_mapTransform.getOrigin().y() - 0.3 + xdistCenter;
    goal.target_pose.pose.position.z = baselink_mapTransform.getOrigin().z();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(northYaw);

    movementClientAsync(goal);
}

void ROSbridge::goSouth()
{
    ROS_INFO("Going north");

    try
    {
        tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tfl.lookupTransform("map", "base_link", ros::Time(0), baselink_mapTransform);
    }
    catch (const tf::TransformException &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR("Transform error: %s", e.what());
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = baselink_mapTransform.getOrigin().x() + 0.3 + ydistCenter;
    goal.target_pose.pose.position.y = baselink_mapTransform.getOrigin().y() + xdistCenter;
    goal.target_pose.pose.position.z = baselink_mapTransform.getOrigin().z();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(northYaw);

    movementClientAsync(goal);
}

void ROSbridge::goWest()
{
    ROS_INFO("Going north");

    try
    {
        tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tfl.lookupTransform("map", "base_link", ros::Time(0), baselink_mapTransform);
    }
    catch (const tf::TransformException &e)
    {
        // std::cerr << e.what() << '\n';
        ROS_ERROR("Transform error: %s", e.what());
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = baselink_mapTransform.getOrigin().x() + 0.3 + ydistCenter;
    goal.target_pose.pose.position.y = baselink_mapTransform.getOrigin().y() + xdistCenter;
    goal.target_pose.pose.position.z = baselink_mapTransform.getOrigin().z();
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(northYaw);

    movementClientAsync(goal);
}

#endif