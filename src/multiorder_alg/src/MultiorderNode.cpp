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
    currentMove_ = -1;

    // No orders yet.
    waitingOrders_ = std::vector<Order>{};

    // Open a publisher for sending waypoints to the robot. 
    cmdPub_ = nh_.advertise<multiorder_alg::batch>("/robot_waypoints", 1);
    // Open a listener for receiving orders from the user. 
    orderSub_ = nh_.subscribe<multiorder_alg::order>("/incoming_orders", 100, &MultiorderNode::orderCallback, this);
    // Open a listener for receiving status updates from the robot.
    statusSub_ = nh_.subscribe<multiorder_alg::robotStatus>("/robot_status", 100, &MultiorderNode::robotStatusCallback, this);

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
    // of the waiting orders. The robot is idling here so 
    // calculate and send a new batch to the next waypoint. 
    if(plannedMoves_.size() == 0) {
        plannedMoves_ = solver.calculateMultiorder(waitingOrders_, robotLocation_);
        waitingOrders_.clear();
        ROS_INFO_STREAM("Current batch of moves finished, making new batch\n");

        // Create a new batch of waypoints and send it to the robot. 
        std::vector<multiorder_alg::waypoint> waypoints; 
        
        for(int i = 0; i < plannedMoves_.size(); i++) {
            multiorder_alg::waypoint w;
            w.nextNode = plannedMoves_[i].node;
            w.action = moveToString_[plannedMoves_[i].action];
            waypoints.push_back(w);
        }

        // The robot starts off executing the 0th order.
        currentMove_ = 0;
        multiorder_alg::batch batch;
        batch.waypoints = waypoints;
        cmdPub_.publish(batch);
    }
    lock_.unlock();
}

void MultiorderNode::robotStatusCallback(const multiorder_alg::robotStatus status) {
    ROS_INFO("Received robot status, robot is at %d\n", status.locationNode);
    lock_.lock();
    // Update the robot's node.
    robotLocation_ = status.locationNode;

    // If the robot has reached the next node in the current batch of 
    // moves then it will naturally go to the next move. 
    if(plannedMoves_.size() > 0 && robotLocation_ == plannedMoves_[currentMove_].node) {
        ROS_INFO("Robot has reached the next node, at %d\n", robotLocation_);
        currentMove_++;

        // If that was the last move then we're done with the batch. 
        if(currentMove_ == plannedMoves_.size()) {
            plannedMoves_.clear();
        }
    }

    // If there's no current batch (i.e. it just finished or the robot has 
    // been idling), then recalculate a new batch and send it to the robot, 
    // if there are any waiting orders.
    if(plannedMoves_.size() == 0 && waitingOrders_.size() > 0) {
        plannedMoves_ = solver.calculateMultiorder(waitingOrders_, robotLocation_);
        waitingOrders_.clear();
        ROS_INFO_STREAM("Current batch of moves finished, making new batch\n");

        // Create a new batch of waypoints and send it to the robot. 
        std::vector<multiorder_alg::waypoint> waypoints; 
        
        for(int i = 0; i < plannedMoves_.size(); i++) {
            multiorder_alg::waypoint w;
            w.nextNode = plannedMoves_[i].node;
            w.action = moveToString_[plannedMoves_[i].action];
            waypoints.push_back(w);
        }

        // The robot starts off executing the 0th order.
        currentMove_ = 0;
        multiorder_alg::batch batch;
        batch.waypoints = waypoints;
        cmdPub_.publish(batch);
    } 
    
    if(plannedMoves_.size() == 0){
        ROS_INFO_STREAM("No orders or waiting moves, robot is now idle.\n");
    } else {
        ROS_INFO_STREAM("Robot still has " << plannedMoves_.size() - currentMove_ << " waypoints left in its batch.\n");
    }

    lock_.unlock();
}

} // namespace Multiorder
