#pragma once

#include <ros/ros.h>
#include <limits>
#include <map>
#include <utility>
#include <vector>

namespace Multiorder {

class MultiorderNode {
public:
    MultiorderNode(ros::NodeHandle& nodeHandle);
    virtual ~MultiorderNode();

private:
    ros::NodeHandle& nh_;

    static constexpr int numNodes_ = 7;
    
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
