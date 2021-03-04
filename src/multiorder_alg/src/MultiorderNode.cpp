#include <multiorder_alg/MultiorderNode.hpp>

namespace Multiorder {

MultiorderNode::MultiorderNode(ros::NodeHandle& nodeHandle,
        std::map<int, std::set<int>>& neighbors, std::vector<std::vector<double>>& weights, 
        int numNodes, double capacity, int robotStartNode) :
    nh_(nodeHandle), 
    numNodes_(numNodes), capacity_(capacity), robotLocation_(robotStartNode),
    solver(MultiorderSolver(neighbors, weights, numNodes, capacity))
{
    // No moves yet. 
    plannedMoves_ = std::vector<Move>{};

    // No orders yet.
    waitingOrders_ = std::vector<Order>{};

    // Open a publisher for sending waypoints to the robot. 
    cmdPub_ = nh_.advertise<multiorder_alg::waypoint>("/robot_waypoints", 1);
    // Open a listener for receiving orders from the user. 
    orderSub_ = nh_.subscribe<multiorder_alg::order>("/incoming_orders", 10, &MultiorderNode::orderCallback, this);
    // Open a listener for receiving status updates from the robot.
    statusSub_ = nh_.subscribe<multiorder_alg::robotStatus>("/robot_status", 10, &MultiorderNode::robotStatusCallback, this);

    // Initialize the move to string mapping for sending out waypoints. 
    moveToString_[PICKUP] = "PICKUP";
    moveToString_[TRANSIT] = "TRANSIT";
    moveToString_[DROPOFF] = "DROPOFF";
}


MultiorderNode::~MultiorderNode() {}


void MultiorderNode::orderCallback(const multiorder_alg::order order) {
    ROS_INFO("Received order %d from %d to %d, with weight %f\n", 
            order.id, order.startNode, order.destNode, order.weight);
    lock_.lock();
    // Enqueue the order for processing.
    waitingOrders_.push_back(Order(order.id, order.startNode, order.destNode, order.weight));

    // If the batch of current moves is done, then start a new batch with all 
    // of the waiting orders. The robot is idling here since there's no current 
    // order (no plannedMoves_[0]) so send it the next waypoint. 
    if(plannedMoves_.size() == 0) {
        plannedMoves_ = solver.calculateMultiorder(waitingOrders_, robotLocation_);
        waitingOrders_.clear();

        multiorder_alg::waypoint w;
        w.nextNode = plannedMoves_[0].node;
        w.action = moveToString_[plannedMoves_[0].action];
        cmdPub_.publish(w);

        // Update the robotLocation_ to match the current move.
        robotLocation_ = plannedMoves_[0].node;
        
        ROS_INFO("Sent robot move to go to %d for %s\n", 
                plannedMoves_[0].node, 
                moveToString_[plannedMoves_[0].action].c_str());
    }
    lock_.unlock();
}

void MultiorderNode::robotStatusCallback(const multiorder_alg::robotStatus status) {
    ROS_INFO("Received robot status, robot is at %d\n", status.locationNode);
    lock_.lock();
        
    multiorder_alg::waypoint w;

    // If the robot is at the wrong location then throw an error. 
    if(robotLocation_ != status.locationNode) {
        ROS_ERROR("Robot is at wrong location %d, should have gone to %d for %s\n", 
                status.locationNode, plannedMoves_[0].node, 
                moveToString_[plannedMoves_[0].action].c_str());
    } else {
        // Otherwise, remove the current planned move, as it's done. 
        plannedMoves_.erase(plannedMoves_.begin());

        // If the current batch of moves is done then 
        // create a new batch with the waiting orders if there
        // are any. 
        if(plannedMoves_.size() == 0 && waitingOrders_.size() > 0) {
            plannedMoves_ = solver.calculateMultiorder(waitingOrders_, robotLocation_);
            waitingOrders_.clear();
        }

        // Send the robot another waypoint if there is one. 
        if(plannedMoves_.size() > 0) {
            w.nextNode = plannedMoves_[0].node;
            w.action = moveToString_[plannedMoves_[0].action];
            cmdPub_.publish(w);

            // Update the robotLocation_ to match the current move.
            robotLocation_ = plannedMoves_[0].node;
        
            ROS_INFO("Sent robot move to go to %d for %s\n", 
                plannedMoves_[0].node, 
                moveToString_[plannedMoves_[0].action].c_str());
        } else {
            if (plannedMoves_.size() || waitingOrders_.size()) {
                ROS_ERROR("Should be no more moves or orders at this \ 
                        block of code but there are.");
            }
            ROS_INFO("Ground station has no more moves or orders, robot will \ 
                    be idle until new orders come in.\n");
        }
    }

    lock_.unlock();
}

} // namespace Multiorder
