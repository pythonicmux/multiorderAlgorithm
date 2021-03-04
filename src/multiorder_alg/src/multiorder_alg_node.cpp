#include <ros/ros.h>
#include <multiorder_alg/MultiorderNode.hpp>
#include "multiorder_alg/order.h"
#include "multiorder_alg/robotStatus.h"
#include "multiorder_alg/waypoint.h"
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

void printMoves(std::vector<Multiorder::Move> sol) {
    for(auto move:sol) {
        if(move.action == Multiorder::DROPOFF) {
            ROS_INFO_STREAM("Dropping off order " << move.id << " at node " << move.node);
        } else if (move.action == Multiorder::PICKUP) {
            ROS_INFO_STREAM("Picking up order " << move.id << " at node " << move.node);
        } else if (move.action == Multiorder::TRANSIT) {
            ROS_INFO_STREAM("In transit at node " << move.node);
        } else {
            ROS_ERROR("Invalid move detected in the sequence of moves.");
        }
    }
}

//// ALGORITHM TESTS

// An easy test with disjoint 2-node paths.
void testEasy(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 0, 1, 1.5));
    input.push_back(Multiorder::Order(1, 2, 3, 0.5));
    input.push_back(Multiorder::Order(2, 5, 6, 2.0));

    auto sol = mn.solver.calculateMultiorder(input, 0);

    if(sol.size() == 0) {
        ROS_ERROR("testEasy failed\n");
    } else {
        printMoves(sol);

        ROS_INFO_STREAM("testEasy passed\n");
    }
}

// An medium test that has some random long paths and short 
// paths.
void testMedium(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 0, 6, 0.1));
    input.push_back(Multiorder::Order(1, 2, 3, 0.5));
    input.push_back(Multiorder::Order(2, 5, 6, 1.9));
    input.push_back(Multiorder::Order(3, 1, 3, 0.8));
    input.push_back(Multiorder::Order(4, 4, 2, 0.7));
    input.push_back(Multiorder::Order(5, 4, 3, 2.0));

    auto sol = mn.solver.calculateMultiorder(input, 4);

    if(sol.size() == 0) {
        ROS_ERROR("testMedium failed\n");
    } else {
        printMoves(sol);

        ROS_INFO_STREAM("testMedium passed\n");
    }
}

// Test that the intermediate path between the furthest-away 
// nodes is correct. Two orders, 3 to 4 and 4 to 3. The robot 
// should correctly find the shortest path from 3 to 4.
void testIntermediatePath(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 4, 3, 0.7));
    input.push_back(Multiorder::Order(1, 3, 4, 1.0));

    auto sol = mn.solver.calculateMultiorder(input, 0);

    if(sol.size() == 0) {
        ROS_ERROR("testIntermediatePath failed\n");
    } else {
        printMoves(sol);

        ROS_INFO_STREAM("testIntermediatePath passed\n");
    }
}

// An impossible test with a 2.1 kg order. Should return no solution. 
void testImpossibleWeight(Multiorder::MultiorderNode& mn) 
{
    std::vector<Multiorder::Order> input;
    input.push_back(Multiorder::Order(0, 0, 1, 2.1));

    auto sol = mn.solver.calculateMultiorder(input, 0);

    if(sol.size() == 0) {
        ROS_INFO_STREAM("testImpossibleWeight failed as intended\n");
    } else {
        printMoves(sol);
        ROS_ERROR("testImpossibleWeight passed when it shouldn't\n");
    }
}

//// GROUND STATION TESTS

// Immediate "goes to" all waypoints it gets.
class TestRobotNode {
public:
    TestRobotNode(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
        // Create robot status publisher and waypoint listener. 
        sPub_ = nh_.advertise<multiorder_alg::robotStatus>("/robot_status", 10);
        wSub_ = nh_.subscribe<multiorder_alg::waypoint>("/robot_waypoints", 10, 
                &TestRobotNode::sendBackLocation, this); 
        ROS_INFO("Test robot up\n");
    }

    ~TestRobotNode() {} 

    void sendBackLocation(const multiorder_alg::waypoint waypoint) {
        ROS_INFO("Test robot got waypoint to go to %d for %s\n", 
                waypoint.nextNode, waypoint.action.c_str());
        multiorder_alg::robotStatus rs;
        rs.locationNode = waypoint.nextNode;
        sPub_.publish(rs);
    }

private:
    ros::NodeHandle& nh_;
    ros::Publisher sPub_;
    ros::Subscriber wSub_;
};


// Give the robot a bunch of orders to pick up from each node 
// on the map and deliver to 
// the dest node. Simulates the actual multiorder node.  
void groundStationTest(ros::NodeHandle& nh, int destNode) { 
    // Create an order publisher.
    ros::Publisher oPub = nh.advertise<multiorder_alg::order>("/incoming_orders", 10);

    // Create some orders that all go to this destination node.
    for(int i = 0; i < 7; i++) {
        multiorder_alg::order msg;
        msg.id = i;
        msg.startNode = i;
        msg.destNode = destNode;
        msg.weight = 0.5;
        oPub.publish(msg);
        ROS_INFO("Sent order %d, going from %d to %d with weight %f\n", 
                msg.id, msg.startNode, destNode, msg.weight);
    }
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

    // Test the multiorder algorithm. 
    ROS_INFO_STREAM("Tests starting...\n");
    testEasy(mn);
    testMedium(mn);
    testIntermediatePath(mn);
    testImpossibleWeight(mn);

    // COMMENT THIS OUT WHEN TESTING - TESTROBOTNODE 
    // PUBLISHES THINGS.
    TestRobotNode testRobot(nodeHandle);
    groundStationTest(nodeHandle, robotStartNode);

    ROS_INFO_STREAM("...tests done. Ready for operation.\n");

    ros::spin();

    return 0;
}
