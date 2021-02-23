#include <multiorder_alg/MultiorderNode.hpp>

namespace Multiorder {

/*
 * The graph of engineering quad roads is as follows: (weights are distances in meters)
 * Wean Turtle             Doherty Main Entrance
 * 0--89.62--1---38.74---2---53.97---3
 * |         |        /  |
 * 32.29    31.68 49.62 34.7           
 * |         |     /     |
 * 4--91.91--5---39.59---6
 * Porter               Baker
 */
MultiorderNode::MultiorderNode(ros::NodeHandle& nodeHandle) : nh_(nodeHandle) {
    // Initialize the graph.
    for(int i = 0; i < numNodes_; i++) {
        neighbors_[i] = std::set<int>{};
        for(int j = 0; j < numNodes_; j++) {
            if(i != j) {
                weights_[i][j] = -1.0;
            } else {
                weights_[i][j] = 0.0;
            }
        }
    }

    neighbors_[0].insert(1);
    weights_[0][1] = 89.62;
    neighbors_[0].insert(4);
    weights_[0][4] = 32.29;

    neighbors_[1].insert(0);
    weights_[1][0] = 89.62;
    neighbors_[1].insert(2);
    weights_[1][2] = 38.74;
    neighbors_[1].insert(5);
    weights_[1][5] = 31.68;

    neighbors_[2].insert(1);
    weights_[2][1] = 38.74;
    neighbors_[2].insert(3);
    weights_[2][3] = 53.97;
    neighbors_[2].insert(5);
    weights_[2][5] = 49.62;
    neighbors_[2].insert(6);
    weights_[2][6] = 34.7;

    neighbors_[3].insert(2);
    weights_[3][2] = 53.97;

    neighbors_[4].insert(0);
    weights_[4][0] = 32.29;
    neighbors_[4].insert(5);
    weights_[4][5] = 91.91;

    neighbors_[5].insert(1);
    weights_[5][1] = 31.68;
    neighbors_[5].insert(2);
    weights_[5][2] = 49.62;
    neighbors_[5].insert(4);
    weights_[5][4] = 91.91;
    neighbors_[5].insert(6);
    weights_[5][6] = 39.59;

    neighbors_[6].insert(2);
    weights_[6][2] = 34.7;
    neighbors_[6].insert(5);
    weights_[6][5] = 39.59;

    precalculateMinTravelTimes();

    // Make sure the precomputations are correct. 
    /*for (int i = 0; i < numNodes_; i++) {
        ROS_INFO_STREAM(i << " is connected to: ");
        for (int j : neighbors_[i]) {
            ROS_INFO_STREAM(j << " ");
        }
        ROS_INFO_STREAM('\n');
    }
 
    for (int i = 0; i < numNodes_; i++) {
        for (int j = 0; j < numNodes_; j++) {
            ROS_INFO_STREAM("d(" << i << ", " << j << ") = " << minTravelTimes_[i][j] << "\n");
        }
    }*/
}

MultiorderNode::~MultiorderNode() {}

void MultiorderNode::precalculateMinTravelTimes() {
    // Calculate the min travel times from this node 
    // with Dijkstra's algorithm.
    for(int source = 0; source < numNodes_; source++) {
        // Each node only needs to be smoothed once, 
        // so keep track of done nodes.
        std::vector<bool> done(numNodes_, false);

        for(int i = 0; i < numNodes_; i++) {
            minTravelTimes_[source][i] = DBL_MAX;
        }
        minTravelTimes_[source][source] = 0;

        for (int i = 0; i < numNodes_; i++) {
            // Get the closest node to the source node for smoothing.
            int closest = -1;
            for(int j = 0; j < numNodes_; j++) {
                if (!done[j] && (closest == -1 || 
                    minTravelTimes_[source][j] < 
                    minTravelTimes_[source][closest])) {
                    closest = j;
                }
            }

            // Only inaccessible nodes left if nothing is reachable anymore.
            if (minTravelTimes_[source][closest] == DBL_MAX) {
                break;
            }

            // Smooth out this node to make sure it's the true min travel time.
            done[closest] = true;
            for(int nbr:neighbors_[closest]) {
                // nbr gets "smoothed" if there's a faster 
                // path to the node via source -> ... -> closest -> nbr.
                if (minTravelTimes_[source][closest] + weights_[closest][nbr] < 
                        minTravelTimes_[source][nbr]) {
                    minTravelTimes_[source][nbr] = minTravelTimes_[source][closest] + 
                        weights_[closest][nbr];
                }

                if (weights_[closest][nbr] < 0.0) {
                    ROS_ERROR("Dijkstra's found that a neighbor has negative weight.");
                }
            } 
        }
    }
}

void MultiorderNode::calculateMultiorder(std::vector<Order> orders) {

} 



} // namespace Multiorder
