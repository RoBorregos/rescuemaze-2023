#include "TfBroadcaster.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle *n = new ros::NodeHandle;

    TfBroadcaster tfb(n);
    tfb.run();
}