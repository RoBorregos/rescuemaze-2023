#include "ROSbridge.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bridgetest");
    ros::NodeHandle *n = new ros::NodeHandle;

    ROSbridge bridge(n);

    // bridge.sendUnitGoal(0);
    // bridge.sendUnitGoal(1);
    // bridge.tcsdata = '1';
    // bridge.sendUnitGoal(2);
    // bridge.sendUnitGoal(3);

    vector<int> walls = bridge.getWalls();

    for (int i = 0; i < walls.size(); i++)
    {
        ROS_INFO("Wall %d: %d", i, walls[i]);
        // cout << walls[i] << endl;
    }
    
    ros::spin();
}