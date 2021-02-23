#pragma once

#include <ros/ros.h>
#include <limits>
#include <map>
#include <vector>

namespace Multiorder {

// The actions the robot can take at each node 
// when processing orders.
enum Actions { PICKUP, DROPOFF };

// The definition of an "order" that the algorithm 
// must process. 
struct Order {
    int id; // Unique order ID
    int S; // Source node
    int D; // Destination node
    double w; // Order weight

    Order(int id, int S, int D, double w) :
        id(id), S(S), D(D), w(w) {}

    // Order must be set-friendly. 
    bool operator<(const Order& rhs) const 
    {
        return id < rhs.id; 
    }
    
    bool operator==(const Order& rhs) const 
    {
        return id == rhs.id; 
    }
    
    bool operator!=(const Order& rhs) const 
    {
        return id != rhs.id; 
    }

    // Nice to have a default copy constructor 
    // for any recursion.
    Order(const Order&) = default;
};

// The definition of a "move" that a robot makes, 
// for the output.
struct Move {
    int node; // Node that the robot has travelled to
    Actions action;
    int id; // Order ID robot is processing
};

class MultiorderNode {
public:
    // Initialize a multiorder simulation with the specified map 
    // and robot parameters. The robot state will reset to the 
    // specified initial state between each call to calculateMultiorder. 
    //
    // numNodes/capacity must be greater than 0, weights must be length [numNodes][numNodes], 
    // robotStartNode must be a valid node and neighbors must be length numNodes, 
    // otherwise this constructor will throw an error.
    MultiorderNode(ros::NodeHandle& nodeHandle,
            std::map<int, std::set<int>>& neighbors, std::vector<std::vector<double>>& weights, 
            int numNodes = 0, double capacity = 2.0, int robotStartNode = 0);

    virtual ~MultiorderNode();

    // calculateMultiorder will print out a satisfactory 
    // order of nodes to go to and actions to perform at each 
    // node in order to deliver all orders, and make sure that 
    // the time between picking up and dropping off each order id
    // is at most 2 * minTravelTimes_[S_id][D_id] (i.e. the minimum time it 
    // takes to go from S to D and back), and that the robot 
    // never picks up more than its capacitys' worth of orders. 
    std::vector<Move> calculateMultiorder(std::vector<Order> orders);

private:
    ros::NodeHandle& nh_;
    
    // Map and robot-defined constants, passed in by the programmer.
    const int numNodes_;
    const double capacity_;
    const int robotStartNode_;

    // This has the neighbors of each node. A node has a neighbor iff 
    // the node and neighbor are connected directly by a road. 
    std::map<int, std::set<int>>& neighbors_;
    // The weight represents the seconds it takes to traverse an edge, 
    // assuming the robot travels at 1 m/s. Because it's 1 m/s, it also 
    // represents the distance in meters between the vertices of the edge.
    std::vector<std::vector<double>>& weights_;

    // This maps each pair of points to the minimum travel time 
    // between them (i.e. the cost of the lowest-cost path between them).
    std::vector<std::vector<double>> minTravelTimes_;
    // This will calculate minTravelTimes_ with the pairwise minimum 
    // travel time between each pair of nodes.
    void precalculateMinTravelTimes();

    // An internal representation of the robot's state, used for 
    // state enumeration and recursion.
    struct Robot {
        int location; // The node the robot is on.
        std::set<Order> currentOrders; // All orders the robot currently has onboard.
        std::map<Order, double> deliveryTimes; // How long the delivery's been taking so far.
        std::set<Order> remainingOrders; // Orders left for the robot to pick up.
        std::vector<Move> moves;
        double capacity; // Remaining capacity 

        Robot(int startNode, double capacity) : 
            location{startNode}, capacity{capacity} {}
        Robot(const Robot& r) {
            location = r.location;
            for(auto x:r.currentOrders) {
                currentOrders.insert(x);
            }
            for(auto x:r.remainingOrders) {
                remainingOrders.insert(x);
            }
            for(auto p:r.deliveryTimes) {
                deliveryTimes[p.first] = p.second;
            }
            for(auto x:r.moves) {
                moves.push_back(x);
            }
            capacity = r.capacity;
        }
        
        ~Robot() {};
    };
    
    // Helper function for recursing down with a partially completed state. 
    std::vector<Move> calculateMultiorder(Robot robot);
};

} // namespace Multiorder
