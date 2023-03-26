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

// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <tf/transform_listener.h>

// #include <nav_msgs/GetMap.h>
// #include <nav_msgs/SetMap.h>
// #include <nav_msgs/OccupancyGrid.h>

#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

#include <nav_main/GetWalls.h>

#include "TfBroadcaster.h"

#include <string>
using namespace std;

#include "ClientScope.h"

#define sendMapGoals true
// #define debugmapgoals true

class ROSbridge
{
private:
    bool debugging;
    bool debugmapgoals;
    ros::NodeHandle *nh;

    geometry_msgs::TransformStamped transformStamped;
    tf::StampedTransform baselink_mapTransform;

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
    ros::Publisher orientationpub;
    ros::Publisher debugpub;

    // Transform broadcaster
    TfBroadcaster tfb;
    // tf2_ros::TransformBroadcaster tfbr;

    // Transform listenerString
    tf::TransformListener tfl;

    // Service clients
    // ros::ServiceClient mapclient;
    ros::ServiceClient moveBaseMapResetClient;
    ros::ServiceClient hsMapReset;

    ros::ServiceClient wallsClient;

    // Action clients
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac;

    // Subscriber callbacks
    void tcscallback(const std_msgs::Char::ConstPtr &msg);
    void bnoxcallback(const std_msgs::Float64::ConstPtr &msg);
    void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
    void localizationCallback(const geometry_msgs::Point::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);

    // Action client callbacks
    void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result);
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
    void activeCb();

    // Action client functions
    void movementClientAsync(move_base_msgs::MoveBaseGoal goal);
    void sendGoal(geometry_msgs::Pose pose);

    void cancelGoal();

    double yawDifference(double yaw1, double yaw2);

public:
    void publishIdealOrientation(int orientation);
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

    double transformYaw;

    double xdistCenter;
    double ydistCenter;

    ROSbridge(ros::NodeHandle *n);

    void clearMap();
    void sendUnitGoal(int movement, int rDirection);
    void sendMapGoal(int movement, int rDirection);
    void sendMapGoalGOAT(int movement, int rDirection);

    void goNorth();
    void goEast();
    void goSouth();
    void goWest();

    vector<int> getWalls();
    void sendKit();
    int getVictims();

    void publishTransform();

    void pubDebug(string msg);
};

ROSbridge::ROSbridge(ros::NodeHandle *n) : ac("move_base", true), tfb(n)
{
    debugging = true;
    debugmapgoals = true;
    started = false;
    xdistCenter = 0;
    ydistCenter = 0;
    tcsdata = '0';
    limitSwitch1 = false;

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
    orientationpub = nh->advertise<std_msgs::Float64>("/ideal_orientation", 1000);

    // // Transform broadcaster
    // transformStamped = geometry_msgs::TransformStamped();
    // transformStamped.header.frame_id = "base_link";
    // transformStamped.child_frame_id = "perfect_position";

    // mapclient = nh->serviceClient<nav_msgs::GetMap>();
    moveBaseMapResetClient = nh->serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    hsMapReset = nh->serviceClient<std_srvs::Trigger>("/reset_map");

    wallsClient = nh->serviceClient<nav_main::GetWalls>("/get_walls");

    ROS_INFO("Created service clients");

    // tfb.run();
    ROS_INFO("Started transform broadcaster");

    // tfl.waitForTransform("base_link", "perfect_position", ros::Time(0), ros::Duration(10.0));
}

// Subscriber callbacks

void ROSbridge::tcscallback(const std_msgs::Char::ConstPtr &msg)
{
    ROS_INFO("I heard: [%c]", msg->data);
    // tcsdata = msg->data;
    tcsdata = 'b';
}

void ROSbridge::bnoxcallback(const std_msgs::Float64::ConstPtr &msg)
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

void ROSbridge::sendMapGoalGOAT(int movement, int rDirection) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "perfect_position";
    if (movement == 0) {
        ROS_INFO("Forward");

        pose.pose.position.x = 0.3;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
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
    movementClientAsync(goal);

}
void ROSbridge::sendUnitGoal(int movement, int rDirection)
{
    if (true) {
        sendMapGoalGOAT(movement, rDirection);

        return;
    }

    if (sendMapGoals)
    {
        sendMapGoal(movement, rDirection);

        return;
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

        switch (rDirection)
        {
        case 0: // North
            tfb.setTransformYaw(yawDifference(northYaw, yaw));
            break;
        case 1: // East
            tfb.setTransformYaw(yawDifference(eastYaw, yaw));
            break;
        case 2: // South
            tfb.setTransformYaw(yawDifference(southYaw, yaw));
            break;
        case 3: // West
            tfb.setTransformYaw(yawDifference(westYaw, yaw));
            break;
        default:
            tfb.setTransformYaw(yawDifference(northYaw, yaw));
            break;
        }

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

        switch (rDirection)
        {
        case 0: // North
            tfb.setTransformYaw(yawDifference(eastYaw, yaw));
            break;
        case 1: // East
            tfb.setTransformYaw(yawDifference(southYaw, yaw));
            break;
        case 2: // South
            tfb.setTransformYaw(yawDifference(westYaw, yaw));
            break;
        case 3: // West
            tfb.setTransformYaw(yawDifference(northYaw, yaw));
            break;

        default:
            break;
        }

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

        switch (rDirection)
        {
        case 0: // North
            tfb.setTransformYaw(yawDifference(westYaw, yaw));
            break;
        case 1: // East
            tfb.setTransformYaw(yawDifference(northYaw, yaw));
            break;
        case 2: // South
            tfb.setTransformYaw(yawDifference(eastYaw, yaw));
            break;
        case 3: // West
            tfb.setTransformYaw(yawDifference(southYaw, yaw));
            break;
        }

        // Calculate angle to get 90 degree turn
        /*   double yawAngle;

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
}

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
    catch(const tf::TransformException& e)
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

void ROSbridge::goNorth()
{
    ROS_INFO("Going north");

    try
    {
        tfl.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tfl.lookupTransform("map", "base_link", ros::Time(0), baselink_mapTransform);
    }
    catch(const tf::TransformException& e)
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
    catch(const tf::TransformException& e)
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
    catch(const tf::TransformException& e)
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
    catch(const tf::TransformException& e)
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

void ROSbridge::publishIdealOrientation(int orientation) {
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
    switch(orientation) {
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