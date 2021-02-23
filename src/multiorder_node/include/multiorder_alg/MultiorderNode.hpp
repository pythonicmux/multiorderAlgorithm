#pragma once

#include <ros/ros.h>
#include <map>
#include <utility>
#include <vector>

namespace Multiorder {

class MultiorderNode {
public:
    MultiorderNode(ros::NodeHandle& nodeHandle);
    virtual ~MultiorderNode();

private:
    static const std::vector<char> nodes_; 
    ros::NodeHandle& nh_;
    std::map<char, std::set<char>> edges_;
    std::map<std::pair<char, char>, double> weights_;
};

} // namespace Multiorder
