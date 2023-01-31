#include <string>
using namespace std;

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "exploration/Trigger.h"

class InstructionServer
{
private:
    string curInstruction;    
    ros::CallbackQueue callQueue;
public:
    InstructionServer(int argc, char **argv);
    bool sendInstruction(exploration::Trigger::Request &req, exploration::Trigger::Response &res);

    void setInstruction(string instruction);
};

InstructionServer::InstructionServer(int argc, char **argv)
{
    ros::init(argc, argv, "instructionServer");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("trigger", InstructionServer::sendInstruction);
    ROS_INFO("Ready to send instruction.");

    curInstruction = "";
}

bool InstructionServer::sendInstruction(exploration::Trigger::Request &req, exploration::Trigger::Response &res)
{
    res.message = curInstruction;
    ROS_INFO("Sending instruction: ");
    ROS_INFO(curInstruction.c_str());

    curInstruction = "";

    return true;
}

void InstructionServer::setInstruction(string instruction)
{
    curInstruction = instruction;

    while (ros::ok() && callQueue.isEmpty())
    {
        ros::Duration(0.2).sleep();
    }
    
    callQueue.callOne(ros::WallDuration(2.0));
}