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
    ros::NodeHandle& nh_;
    std::map<char, std::vector<std::pair<char, int>>> graph_;
};

} // namespace Multiorder
