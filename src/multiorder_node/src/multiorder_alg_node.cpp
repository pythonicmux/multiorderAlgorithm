#include <ros/ros.h>
#include <multiorder_alg/MultiorderNode.hpp>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "multiorder_alg_node");
    ros::NodeHandle nodeHandle;

    Multiorder::MultiorderNode mn(nodeHandle);

    ros::spin();
    return 0;
}
