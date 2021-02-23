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

// An medium test that has some random long paths and short 
// paths.
bool testMedium(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 0, 6, 0.1));
    input.push_back(Multiorder::Order(1, 2, 3, 0.5));
    input.push_back(Multiorder::Order(2, 5, 6, 1.9));
    input.push_back(Multiorder::Order(3, 1, 3, 0.8));
    input.push_back(Multiorder::Order(4, 4, 2, 0.7));
    input.push_back(Multiorder::Order(5, 4, 3, 2.0));

    auto sol = mn.calculateMultiorder(input);

    if(sol.size() == 0) {
        ROS_ERROR("testMedium failed\n");
    } else {
        for(auto move:sol) {
            if(move.action == Multiorder::DROPOFF) {
                ROS_INFO_STREAM("Dropping off order " << move.id << " at " << move.node);
            } else {
                ROS_INFO_STREAM("Picking up order " << move.id << " at " << move.node);
            }
        }

        ROS_INFO_STREAM("testMedium passed\n");
    }
}

// An impossible test with a 2.1 kg order. Should return nothing.
bool testImpossibleWeight(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 0, 1, 2.1));

    auto sol = mn.calculateMultiorder(input);

    if(sol.size() == 0) {
        ROS_INFO_STREAM("testImpossibleWeight failed as intended\n");
    } else {
        ROS_ERROR("testImpossibleWeight passed when it shouldn't\n");
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "multiorder_alg_node");
    ros::NodeHandle nodeHandle;

    Multiorder::MultiorderNode mn(nodeHandle);

    // Test the multiorder algorithm. 
    testEasy(mn);
    testMedium(mn);
    testImpossibleWeight(mn);

    ros::spin();
    return 0;
}
