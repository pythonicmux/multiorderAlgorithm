#include <multiorder_alg/MultiorderNode.hpp>

namespace Multiorder {
/*
 * The graph of engineering quad roads is as follows: (weights are distances in meters)
 * Wean Turtle             Doherty Main Entrance
 * A--89.62--B---38.74---C---53.97---D
 * |         |        /  |
 * 32.29    31.68 49.62 34.7           
 * |         |     /     |
 * E--91.91--F---39.59---G
 * Porter               Baker
 */
MultiorderNode::MultiorderNode(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
}

MultiorderNode::~MultiorderNode() {}

} // namespace Multiorder
