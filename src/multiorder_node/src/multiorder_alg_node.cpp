#include <ros/ros.h>
#include <multiorder_alg/MultiorderNode.hpp>
#include <vector>

// An easy test with disjoint 2-node paths.
bool testEasy(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 0, 1, 1.5));
    input.push_back(Multiorder::Order(1, 2, 3, 0.5));
    input.push_back(Multiorder::Order(2, 5, 6, 2.0));

    auto sol = mn.calculateMultiorder(input);

    if(sol.size() == 0) {
        ROS_ERROR("testEasy failed\n");
    } else {
        for(auto move:sol) {
            if(move.action == Multiorder::DROPOFF) {
                ROS_INFO_STREAM("Dropping off order " << move.id << " at " << move.node);
            } else {
                ROS_INFO_STREAM("Picking up order " << move.id << " at " << move.node);
            }
        }

        ROS_INFO_STREAM("testEasy passed\n");
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "multiorder_alg_node");
    ros::NodeHandle nodeHandle;

    Multiorder::MultiorderNode mn(nodeHandle);

    // Test the multiorder algorithm. 
    testEasy(mn);

    ros::spin();
    return 0;
}
