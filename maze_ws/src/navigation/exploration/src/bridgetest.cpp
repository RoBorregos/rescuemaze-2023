#include "ROSbridge.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bridgetest");
    ros::NodeHandle *n = new ros::NodeHandle;

    ROSbridge bridge(n);

    while (ros::ok() && !bridge.started)
    {
        // clear maps
        ros::spinOnce();

        ROS_INFO("Clearing map");
        bridge.clearMap();

        ros::Duration(3).sleep();
    }
    // define service server
    // ros::ServiceServer service = n->advertiseService("clearMap", &ROSbridge::getWalls,);

    // while (ros::ok() && !bridge.started)
    // {
    //     ros::spinOnce();
    // }

    // int dir = 0;
    // bridge.publishIdealOrientation(dir);

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     // ROS_INFO("North yaw is %f", bridge.northYaw);
    //     // ROS_INFO("East yaw is %f", bridge.eastYaw);
    //     // ROS_INFO("South yaw is %f", bridge.southYaw);
    //     // ROS_INFO("West yaw is %f", bridge.westYaw);
    //     // ROS_INFO("Current yaw is %f", bridge.yaw);

    //     // ROS_INFO("dir is %d", dir);

    //     // bridge.sendUnitGoal(1, dir);
    //     // dir++;
    //     // if (dir > 3)
    //     // {
    //     //     dir = 0;
    //     // }
    //     // bridge.publishTransform();
    //     // bridge.sendUnitGoal(3, dir);
    //     // dir--;


    //     // if (dir < 0)
    //     // {
    //     //     dir = 3;
    //     // }
    //     // bridge.publishIdealOrientation(dir);

    //     // bridge.sendUnitGoal(1, dir);
    //     // dir++;
    //     // bridge.sendUnitGoal(3, dir);
    //     // dir--;
    //     // bridge.sendUnitGoal(3, dir);
    //     // dir--;
    //     // bridge.sendUnitGoal(1, dir);
    //     // dir++;
    //     // bridge.sendUnitGoal(3, dir);
    //     // dir--;
    //     // bridge.sendUnitGoal(1, dir);
    //     // dir++;
    //     // bridge.sendUnitGoal(3, dir);
    //     // dir--;
    //     // ros::Duration(1).sleep();
    // }

    // bridge.sendUnitGoal(1, 0);
    // bridge.ac.waitForResult();

    // bridge.sendUnitGoal(4, 0);
    // bridge.sendUnitGoal(1);
    // bridge.tcsdata = '1';
    // bridge.sendUnitGoal(2);
    // bridge.sendUnitGoal(3);

    // vector<int> walls = bridge.getWalls();

    // for (int i = 0; i < walls.size(); i++)
    // {
    //     ROS_INFO("Wall %d: %d", i, walls[i]);
    //     // cout << walls[i] << endl;
    // }

    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     ROS_INFO("yaw: %f", bridge.yaw);
    // }

    ros::spin();
}