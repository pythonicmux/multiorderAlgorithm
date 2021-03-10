#include <ros/ros.h>
#include <multiorder_alg/MultiorderNode.hpp>
#include "multiorder_alg/batch.h"
#include "multiorder_alg/order.h"
#include "multiorder_alg/robotStatus.h"
#include "multiorder_alg/waypoint.h"
#include <mutex>
#include <set>
#include <vector>

//// CMU TEST MAP

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
    // Allocate memory for the graph. 
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

    // Initialize the graph (weights and neighbors).
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

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "multiorder_alg_node");
    ros::NodeHandle nodeHandle;

    // Initialize the graph of CMU to pass in. 
    initializeCMU();
    
    // Get the starting node.
    int robotStartNode = 0;
    if(nodeHandle.getParam(ros::this_node::getNamespace() + ros::this_node::getName() + 
                "/robot_start_node", robotStartNode) == false) {
        ROS_ERROR("Could not get robot starting node.");
    }

    // Initialize the multiorder node with the map of CMU, with 
    // 7 nodes, a capacity of 2kg, and starting at the node 0 for tests.
    Multiorder::MultiorderNode mn(nodeHandle, neighbors, weights, 
            7, 2.0, robotStartNode);

    ros::spin();

    return 0;
}
