#include "ros/ros.h"

#include <string>
using namespace std;

// Move Base
#include "move_base_msgs/MoveBaseAction.h"
// #include "move_base_msgs/MoveBaseGoal.h"
#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

struct ClientScope
{
    // move_base_msgs::MoveBaseFeedbackConstPtr &feedback;
    string feedback;
    uint8_t result;
    string textRes;
    // int status;
    uint8_t status;
    bool resultReceived;
    bool startedGoal;

    // Constructor
    ClientScope() : feedback(""), result(10), resultReceived(false) {}
};

ClientScope scope;

void doneCb(const actionlib::SimpleClientGoalState &state, const move_base_msgs::MoveBaseResult::ConstPtr &result)
{
    ROS_INFO_STREAM("Finished in state " << state.toString());
    // ROS_INFO_STREAM("Answer: " << result->sequence.back());

    scope.result = state.state_;
    scope.textRes = state.getText();

    // scope.result = result->status.goal_id.id;
    scope.resultReceived = true;
    scope.startedGoal = false;
}

// void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
    // ROS_INFO_STREAM("Feedback x: " << feedback->base_position.pose.position.x);
    // ROS_INFO_STREAM("Feedback y: " << feedback->base_position.pose.position.y);
    // ROS_INFO_STREAM("Feedback z: " << feedback->base_position.pose.position.z);
    // ROS_INFO_STREAM("GoalID: " << feedback->status.goal_id.id);
    // ROS_INFO_STREAM("x: " << feedback->feedback.base_position.pose.position.x);
    // ROS_INFO_STREAM("y: " << feedback->feedback.base_position.pose.position.y);
    // ROS_INFO_STREAM("z: " << feedback->feedback.base_position.pose.position.z);
    // ROS_INFO_STREAM("Got Feedback of length " << feedback->sequence.size());

    scope.feedback = feedback->base_position.pose.position.x;

    // scope.feedback = feedback->.goal_id.id;
}

void activeCb()
{
    ROS_INFO("Goal just went active");
    scope.resultReceived = false;
    scope.startedGoal = true;
}

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
    // ROS_INFO_STREAM("Status: " << msg->goal_id.id);
    ROS_INFO("Status: %d", msg->status_list[0].status);
    if (scope.startedGoal)
        scope.status = msg->status_list[0].status;
    // ROS_INFO_STREAM("Status: " << msg->status_list[0].text);
    // ROS_INFO_STREAM("Status: " << msg->status_list[0].goal_id.id);
}

int getColor()
{
    return 0;
}

void recovery()
{
    ROS_INFO("Recovery");
}

void movementClientAsync(move_base_msgs::MoveBaseGoal goal)
{
    ROS_INFO("movementClientAsync");

    // tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        ros::spinOnce();
    }

    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    scope.resultReceived = false;
    scope.status = actionlib_msgs::GoalStatus::PENDING;

    // ros::spinOnce();

    while (ros::ok() && !scope.resultReceived && 
        scope.status != actionlib_msgs::GoalStatus::PREEMPTED && 
        scope.status != actionlib_msgs::GoalStatus::SUCCEEDED && 
        scope.status != actionlib_msgs::GoalStatus::ABORTED && 
        scope.status != actionlib_msgs::GoalStatus::REJECTED && 
        scope.status != actionlib_msgs::GoalStatus::RECALLED)
    
    //(scope.status == actionlib_msgs::GoalStatus::ACTIVE || scope.status == actionlib_msgs::GoalStatus::PENDING || scope.status == actionlib_msgs::GoalStatus::RECALLED || scope.status == actionlib_msgs::GoalStatus::RECALLING))
    {
        ROS_INFO("in movement client async loop, Status: %d", scope.status);
        ROS_INFO("resultReceived: %d", scope.resultReceived);
        
        if (false)
        {
            ac.cancelAllGoals();
            recovery();

            break;
        }

        ros::spinOnce();
    }

    ROS_INFO("movementClientAsync result: %d", scope.status);
    return;
}

void sendGoal(geometry_msgs::Pose pose)
{
    ROS_INFO("sendGoal");

    move_base_msgs::MoveBaseGoal goal;

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "base_link";

    poseStamped.pose = pose;

    goal.target_pose = poseStamped;
    movementClientAsync(goal);
}

// void handleUnitMovements(exploration::Trigger::Request &req, exploration::Trigger::Response &res)
void handleUnitMovements(int movement)
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client");
    ros::NodeHandle nh;

    ros::Subscriber status_sub = nh.subscribe("move_base/status", 1, &statusCallback);
    // ros::spinOnce();

    handleUnitMovements(0);
    while (ros::ok() && !scope.resultReceived)
    {
        ros::spinOnce();

        ROS_INFO("Result received: %d", scope.resultReceived);
        ROS_INFO("Status: %d", scope.status);
    }
    
    // while (ros::ok() && !scope.resultReceived && scope.status != actionlib_msgs::GoalStatus::SUCCEEDED && scope.status != actionlib_msgs::GoalStatus::ABORTED && scope.status != actionlib_msgs::GoalStatus::REJECTED && scope.status != actionlib_msgs::GoalStatus::RECALLED)
    // {
    //     ROS_INFO_STREAM("Status: " << scope.status);
    //     ros::spinOnce();
    // }
    // ros::Duration(5).sleep();
    handleUnitMovements(3);
    // while (ros::ok() && scope.resultReceived && scope.status != actionlib_msgs::GoalStatus::SUCCEEDED && scope.status != actionlib_msgs::GoalStatus::ABORTED && scope.status != actionlib_msgs::GoalStatus::REJECTED && scope.status != actionlib_msgs::GoalStatus::RECALLED)
    // {
    //     ROS_INFO_STREAM("Status: " << scope.status);
    //     ros::spinOnce();
    // } // ros::Duration(2).sleep();
    // handleUnitMovements(2);

    ros::spin();

    return 0;
}