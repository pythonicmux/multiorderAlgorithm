#pragma once

#include <ros/ros.h>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace Multiorder {

// The definition of an "order" that the algorithm 
// must process. 
struct Order {
    int id; // Order ID
    int S; // Source node
    int D; // Destination node
    int w; // Order weight
};

// The actions the robot can take at each node 
// when processing orders.
enum Actions { PICKUP, DROPOFF };

class MultiorderNode {
public:
    MultiorderNode(ros::NodeHandle& nodeHandle);
    virtual ~MultiorderNode();

    // calculateMultiorder will print out a satisfactory 
    // order of nodes to go to and actions to perform at that 
    // node in order to deliver all orders, and make sure that 
    // the time between picking up and dropping off each order id
    // is at most 2 * minTravelTimes_[S_id][D_id]. 
    void calculateMultiorder(std::vector<Order> orders);

    // Map and robot-defined constants.
    static constexpr int numNodes_ = 7;
    static constexpr double capacity_ = 2.0;
    
private:
    ros::NodeHandle& nh_;

    // This has the neighbors of each node. A node has a neighbor iff 
    // the node and neighbor are connected directly by a road. 
    std::map<int, std::set<int>> neighbors_;
    // The weight represents the seconds it takes to traverse an edge, 
    // assuming the robot travels at 1 m/s. Because it's 1 m/s, it also 
    // represents the distance in meters between the vertices of the edge.
    double weights_[numNodes_][numNodes_];

    // This maps each pair of points to the minimum travel time 
    // between them (i.e. the cost of the lowest-cost path between them).
    double minTravelTimes_[numNodes_][numNodes_];
    // This will calculate minTravelTimes_ with the pairwise minimum 
    // travel time between each pair of nodes.
    void precalculateMinTravelTimes();
};

} // namespace Multiorder
