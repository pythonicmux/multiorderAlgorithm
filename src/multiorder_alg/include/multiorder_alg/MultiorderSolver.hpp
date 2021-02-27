#pragma once

#include "multiorder_alg/order.h"
#include "multiorder_alg/waypoint.h"
#include <ros/ros.h>
#include <limits>
#include <map>
#include <mutex>
#include <vector>

namespace Multiorder {

// The actions the robot can take at each node 
// when processing orders. Either pickup, dropoff, 
// or just in transit. 
enum Actions { PICKUP, DROPOFF, TRANSIT };

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

class MultiorderSolver {
public:
    // Initialize a multiorder algorithm solver with the specified map 
    // and robot parameters. 
    //
    // The graph and robot state passed in must be valid: 
    // numNodes/capacity must be greater than 0, weights must be length [numNodes][numNodes], 
    // otherwise this constructor will throw an error.
    MultiorderSolver(
            std::map<int, std::set<int>>& neighbors, std::vector<std::vector<double>>& weights, 
            int numNodes = 0, double capacity = 2.0);

    virtual ~MultiorderSolver();

    // calculateMultiorder will print out a satisfactory 
    // order of nodes to go to following the edges and actions to perform at each 
    // node in order to deliver all orders, and make sure that 
    // the time between picking up and dropping off each order id
    // is at most 2 * minTravelTimes_[S_id][D_id] (i.e. the minimum time it 
    // takes to go from S to D and back), and that the robot 
    // never picks up more than its capacitys' worth of orders. The 
    // robot will start at node robotStartNode.
    //
    // Each order must contain a valid source/destination node and a non-negative capacity, 
    // and robotStartNode must be a valid node. 
    // Otherwise, the algorithm will have undefined behavior. 
    std::vector<Move> calculateMultiorder(std::vector<Order> orders, int robotStartNode);

    // orderCallback proceses an order and queues it for processing. 
    void orderCallback(const multiorder_alg::order order);

private:
    // Map and robot-defined constants, passed in by the programmer.
    const int numNodes_;
    const double capacity_;

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
    // This contains the shortest paths between i and j for all nodes i and 
    // j, and the paths DO NOT contain the start and end vertices i and j.
    std::vector<std::vector<std::vector<int>>> shortestPaths_;

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
        double remainingCapacity; // Remaining capacity 

        Robot(int startNode, double capacity) : 
            location{startNode}, remainingCapacity{capacity} {}
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
            remainingCapacity = r.remainingCapacity;
        }
        
        ~Robot() {};
    };
    
    // Helper function for recursing down with a partially completed state. 
    std::vector<Move> calculateMultiorder(Robot robot);
};

} // namespace Multiorder
