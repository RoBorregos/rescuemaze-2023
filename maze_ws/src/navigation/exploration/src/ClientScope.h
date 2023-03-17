#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

struct ClientScope
{
    // move_base_msgs::MoveBaseFeedbackConstPtr &feedback;
    string xfeedback;
    string yfeedback;
    string zfeedback;

    uint8_t result;
    string textRes;
    // int status;
    uint8_t status;
    bool resultReceived;
    bool startedGoal;

    // Constructor
    ClientScope() : result(10), resultReceived(false) {}
};
