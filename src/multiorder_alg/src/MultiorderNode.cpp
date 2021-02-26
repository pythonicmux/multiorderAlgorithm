#include <multiorder_alg/MultiorderNode.hpp>

namespace Multiorder {

MultiorderNode::MultiorderNode(ros::NodeHandle& nodeHandle,
        std::map<int, std::set<int>>& neighbors, std::vector<std::vector<double>>& weights, 
        int numNodes, double capacity, int robotStartNode) :
    nh_(nodeHandle), neighbors_(neighbors), 
    weights_(weights), numNodes_(numNodes), capacity_(capacity), robotLocation_(robotStartNode)
{
    // No moves yet. 
    plannedMoves_ = std::vector<Move>{};
    
    // Check to make sure it's a valid graph. 
    if (numNodes_ <= 0 || capacity <= 0.0 ||
            weights_.size() != numNodes_ || weights_[0].size() != numNodes || 
            neighbors_.size() != numNodes) {
        ROS_ERROR("Invalid graph or robot state parameter passed into MultiorderNode.");
    }

    // Allocate memory for minTravelTimes_. 
    for (int i = 0; i < numNodes_; i++) {
        std::vector<double> row(numNodes_);
        minTravelTimes_.push_back(row); 
    }

    // Precalculate all shortest distances/times to 
    // make the multiorder calculation easier. 
    precalculateMinTravelTimes();
    
    // Make sure the precomputations are correct. 
    for (int i = 0; i < numNodes_; i++) {
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
    }

    // Open a publisher for sending waypoints to the robot. 
    cmdPub_ = nh_.advertise<multiorder_alg::waypoint>("/robot_waypoints", 1);
    // Open a listener for receiving orders from the user. 
    orderSub_ = nh_.subscribe<multiorder_alg::order>("/incoming_orders", 10, &MultiorderNode::orderCallback, this);
}


MultiorderNode::~MultiorderNode() {}


void MultiorderNode::orderCallback(const multiorder_alg::order order) {
    lock_.lock();
    // Enqueue the order for processing.
    waitingOrders_.push_back(Order(order.id, order.startNode, order.destNode, order.weight));

    // If the batch of current moves is done, then start a new batch with all 
    // of the waiting orders.
    if(plannedMoves_.size() == 0) {
        plannedMoves_ = calculateMultiorder(waitingOrders_, robotLocation_);
        waitingOrders_.clear();
    }
    lock_.unlock();
}


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


std::vector<Move> MultiorderNode::calculateMultiorder(std::vector<Order> orders, int robotStartNode) {
    // Check that the robot starting point is valid. 
    if (robotStartNode < 0 || robotStartNode > numNodes_) {
        ROS_ERROR("Invalid robot starting node input.");
    }
    // Check that all of the orders are valid. 
    for(auto order:orders) {
        if (order.w < 0 || order.S < 0 || order.S > numNodes_ || 
                order.D < 0 || order.D > numNodes_) {
            ROS_ERROR("User has placed an invalid order to the algorithm.");
        }
    }
    // Initialize the robot start state to the user's passed-in specification. 
    Robot r(robotStartNode, capacity_);
    // Give it the order list to search for a satisfactory plan. 
    for (Order o:orders) {
        r.remainingOrders.insert(o);
    }

    return calculateMultiorder(r);
}

// Try every possible next move that can still satisfy the constraints 
// and backtrack if needed, until either a satisfactory series of moves 
// is found or it's impossible to proceed.
std::vector<Move> MultiorderNode::calculateMultiorder(Robot r) {
    // If we're in an invalid state then return nothing, i.e. it's 
    // impossible given our circumstances.
    // Capacity must be nonnegative, and the delivery times map must be 
    // the same size as the current orders vector. 
    if(r.remainingCapacity < 0 || r.deliveryTimes.size() != r.currentOrders.size()) {
        return std::vector<Move>{};
    }
    // The deliveries must be <= 2*min time to travel there and back.
    for(auto p:r.deliveryTimes) {
        if(p.second > 2*minTravelTimes_[p.first.S][p.first.D]) {
            return std::vector<Move>{};
        }
    }

    // We can either try picking up an order or not picking up any orders.
    for (auto order:r.remainingOrders) {
        // If the order can fit then we try travelling to 
        // the order's start and adding it to the robot's current orders.
        if(order.w <= r.remainingCapacity) {
            Robot next(r);
            next.location = order.S;
            // The robot travels to order.S and time passes.
            for (auto it = r.deliveryTimes.begin(); it != r.deliveryTimes.end(); it++) {
                it->second += minTravelTimes_[r.location][order.S];
            }

            // The robot takes this order, and updates its state.  
            next.remainingOrders.erase(order);
            next.currentOrders.insert(order);
            next.deliveryTimes[order] = 0.0;
            next.remainingCapacity -= order.w;

            next.moves.push_back(Move{order.S, PICKUP, order.id});

            // See if this next state yields a solution.
            auto potentialSol = calculateMultiorder(next);
            if(potentialSol.size()) {
                return potentialSol;
            }
        } 
    }

    // We can try dropping off an order or not dropping off any orders. 
    for (auto order:r.currentOrders) {
        Robot next(r);
        next.location = order.D;
        // The robot travels to order.D and time passes.
        for (auto it = r.deliveryTimes.begin(); it != r.deliveryTimes.end(); it++) {
            it->second += minTravelTimes_[r.location][order.D];
        }

        // The robot drops off this order, and updates its state.  
        next.currentOrders.erase(order);
        next.deliveryTimes.erase(order);
        next.remainingCapacity += order.w;

        next.moves.push_back(Move{order.D, DROPOFF, order.id});

        // See if this next state yields a solution.
        auto potentialSol = calculateMultiorder(next);
        if(potentialSol.size()) {
            return potentialSol;
        }
    }

    // Make sure the state of the robot is intact.
    if(r.remainingCapacity < 0 || r.deliveryTimes.size() != r.currentOrders.size()) {
        return std::vector<Move>{};
    }

    // If there are no valid actions left then either the robot 
    // has completed all orders, which means we're done, 
    // or the robot cannot complete any orders onwards from this state, 
    // so we have an unsuccessful state.
    if(r.remainingOrders.size() > 0) {
        return std::vector<Move>{};
    } else {
        return r.moves;
    }
    
}

} // namespace Multiorder
