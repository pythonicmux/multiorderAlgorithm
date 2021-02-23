#include <ros/ros.h>
#include <multiorder_alg/MultiorderNode.hpp>
#include <set>
#include <vector>

static const int numNodes = 7;
std::map<int, std::set<int>> neighbors;
std::vector<std::vector<double>> weights;

/*
 * The graph of engineering quad roads is as follows: (Weights are distances in meters. Assuming
 * the robot goes 1m/s this weight is also measured in seconds as time.)
 * Wean Turtle             Doherty Main Entrance
 * 0--89.62--1---38.74---2---53.97---3
 * |         |        /  |
 * 32.29    31.68 49.62 34.7           
 * |         |     /     |
 * 4--91.91--5---39.59---6
 * Porter               Baker
 */
void initializeCMU() {
    // Initialize the graph (weights and neighbors).
    for(int i = 0; i < numNodes; i++) {
        neighbors[i] = std::set<int>{};
        weights.push_back(std::vector<double>{});
        for(int j = 0; j < numNodes; j++) {
            if(i != j) {
                weights[i].push_back(-1.0);
            } else {
                weights[i].push_back(0.0);
            }
        }
    }

    neighbors[0].insert(1);
    weights[0][1] = 89.62;
    neighbors[0].insert(4);
    weights[0][4] = 32.29;

    neighbors[1].insert(0);
    weights[1][0] = 89.62;
    neighbors[1].insert(2);
    weights[1][2] = 38.74;
    neighbors[1].insert(5);
    weights[1][5] = 31.68;

    neighbors[2].insert(1);
    weights[2][1] = 38.74;
    neighbors[2].insert(3);
    weights[2][3] = 53.97;
    neighbors[2].insert(5);
    weights[2][5] = 49.62;
    neighbors[2].insert(6);
    weights[2][6] = 34.7;

    neighbors[3].insert(2);
    weights[3][2] = 53.97;

    neighbors[4].insert(0);
    weights[4][0] = 32.29;
    neighbors[4].insert(5);
    weights[4][5] = 91.91;

    neighbors[5].insert(1);
    weights[5][1] = 31.68;
    neighbors[5].insert(2);
    weights[5][2] = 49.62;
    neighbors[5].insert(4);
    weights[5][4] = 91.91;
    neighbors[5].insert(6);
    weights[5][6] = 39.59;

    neighbors[6].insert(2);
    weights[6][2] = 34.7;
    neighbors[6].insert(5);
    weights[6][5] = 39.59;
}

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

    initializeCMU();

    // Initialize the multiorder node with the map of CMU, with 
    // 7 nodes, a capacity of 2kg, and starting at node 0. 
    Multiorder::MultiorderNode mn(nodeHandle, neighbors, weights, 
            7, 2.0, 0);

    // Test the multiorder algorithm. 
    testEasy(mn);
    testMedium(mn);
    testImpossibleWeight(mn);

    ros::spin();

    return 0;
}
