#pragma once

#include <ros/ros.h>

namespace Multiorder {

class MultiorderNode {
public:
    MultiorderNode(ros::NodeHandle& nodeHandle);
    virtual ~MultiorderNode();

private:
    ros::NodeHandle& nh_;
};

} // namespace Multiorder
