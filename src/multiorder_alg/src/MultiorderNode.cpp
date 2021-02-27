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
}


MultiorderNode::~MultiorderNode() {}


void MultiorderNode::orderCallback(const multiorder_alg::order order) {
    lock_.lock();
    // Enqueue the order for processing.
    waitingOrders_.push_back(Order(order.id, order.startNode, order.destNode, order.weight));

    // If the batch of current moves is done, then start a new batch with all 
    // of the waiting orders.
    if(plannedMoves_.size() == 0) {
        plannedMoves_ = solver.calculateMultiorder(waitingOrders_, robotLocation_);
        waitingOrders_.clear();
    }
    lock_.unlock();
}


} // namespace Multiorder
