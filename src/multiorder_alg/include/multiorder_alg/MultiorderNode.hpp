#pragma once

#include "multiorder_alg/batch.h"
#include "multiorder_alg/order.h"
#include "multiorder_alg/robotStatus.h"
#include "multiorder_alg/waypoint.h"

#include "multiorder_alg/MultiorderSolver.hpp"

#include <ros/ros.h>

#include <map>
#include <mutex>
#include <vector>

namespace Multiorder {

class MultiorderNode {
public:
    // Initialize a multiorder planner with the specified map 
    // and robot parameters. It keeps track of the robot location 
    // between each order and uses the multiorder solver to plan 
    // "moves", i.e. waypoints and actions, for the robot. 
    //
    // The graph and robot state passed in must be valid: 
    // numNodes/capacity must be greater than 0, weights must be length [numNodes][numNodes], 
    // robotStartNode must be a valid node and neighbors must be length numNodes, 
    // otherwise this constructor will throw an error.
    MultiorderNode(ros::NodeHandle& nodeHandle,
            std::map<int, std::set<int>>& neighbors, std::vector<std::vector<double>>& weights, 
            int numNodes = 0, double capacity = 2.0, int robotStartNode = 0);

    virtual ~MultiorderNode();

    // orderCallback processes an order and queues it for processing. If there 
    // is no current batch of moves for the robot then it calculates and 
    // sends the robot a new batch of waypoints. 
    void orderCallback(const multiorder_alg::order order);

    // robotStatusCallback receives a robot status update/heartbeat and 
    // keeps track of the robot's location. 
    //
    // It tracks where the robot is in its current batch of moves, 
    // and once the robot is done, it calculates and 
    // sends the robot a new batch of moves, if there are 
    // waiting orders to make a new batch with. 
    void robotStatusCallback(const multiorder_alg::robotStatus status);

    // A solver to process orders and turn them into moves. 
    // Available for public use because of immutability and testing. 
    MultiorderSolver solver;

private:
    ros::NodeHandle& nh_;
    // cmdPub_ will send waypoints to the robot.
    ros::Publisher cmdPub_;
    // orderSub_ will listen for orders from the user. 
    ros::Subscriber orderSub_;
    // statusSub_ will listen to status updates from the robot. 
    ros::Subscriber statusSub_;

    // There will be multiple callbacks modifying robotLocation_ 
    // and plannedMoves_ so we need a lock to protect them. 
    std::mutex lock_;

    // A list of orders that the robot is currently fulfilling and 
    // has not started delivering yet. These orders will start 
    // being delivered once all the currently planned moves are done. 
    std::vector<Order> waitingOrders_;

    // An ordered list of the moves that the robot should do to 
    // fulfill its orders on time. This is the current batch 
    // of moves that is entirely sent to the robot, and the 
    // ground station keeps track of which move the robot is on.
    std::vector<Move> plannedMoves_;
    // The index of the current move in plannedMoves_ that 
    // the robot is executing. This is -1 if the robot is idle.
    int currentMove_;

    // The current location of the robot most recently sent by 
    // a heartbeat (multiorder_alg::robotStatus), set in 
    // the robotStatusCallback function.
    int robotLocation_;
    
    // Map and robot-defined constants, passed in by the programmer.
    const int numNodes_;
    const double capacity_;

    std::map<Actions, std::string> moveToString_;
};

} // namespace Multiorder
