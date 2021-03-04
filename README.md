# Overview

This is a proof-of-concept for an algorithm that will act as a global planner for an
autonomous delivery robot (currently this is just a constraint satisfaction problem 
until I find some kind of cost function to make it an optimization problem). I've also implemented 
an online global planner that uses this algorithm, which can service and plan orders for a robot to deliver. 
I'm designing and implementing this for 18-500 ECE Senior Capstone at Carnegie Mellon, Spring 2021. 
I'm on Team C9 (GrubTub). 

## The problem is as follows:

The robot has a total capacity of C, and is at node S in an undirected, weighted graph G=(V,E). The 
weight of an edge corresponds with the time the robot takes to travel across the edge.

We also precompute shortest paths over G to get a function d(S,D) that can compute the shortest distance of any path from S to D for any two 
nodes in the graph S and D.

The input is a list of orders {(id, (S,D), w), ...}. 

id is the unique identification string of the order. 

S is the starting node, D is the destination node.

w is the weight of the order.

The robot can only fit up to C capacity at once, and can deliver/pick up multiple deliveries 
at the same location, if they all fit. We assume pickups and dropoffs take 0 time.

<b> 
The output must be an ordered list {(S, action, id), ...},
</b> where action is in 
{Pickup, Dropoff, Transit}. Pickup means the robot picks up the order id which starts at node S, 
and Dropoff means the robot drops off the order id which ends at node S. Transit means that 
this move is to an intermediate node that is on the way to processing order id in some way.

For each order id, (S, Pickup, id) must be before (D, Dropoff, id) in the list, and between 
each Pickup and Dropoff there must be a valid sequence of Transit moves to get to the node via 
the edges of the graph. 

<b> 
This output is correct if and only if, each order is picked up, and then dropped off at a later time, 
  and for each order (id, (S,D), w, t), 
the sum of weights the robot travels through from (S, Pickup, id) to (D, Dropoff, id) in the list is 
less than or equal to 2*d(S,D) (i.e. the delivery time is less than the human round trip time, which 
is just the time it takes a human walking from S to D and back to S), and at any point in time the 
robot carries no more than C total capacity. 
</b>

## The solution, currently:

The user specifies a graph and initial robot state for the MultiorderNode to calculate 
its moves on. 

When processing an input list of orders, `MultiorderNode::calculateMultiorder` will create an 
internal robot "state" and backtrack on all possible moves that the robot can do given its current state. 
It recurses and views all possible next states to try to find a solution. 

### Complexity

The state space is all possible robot states, i.e. all possible combos of current orders with elapsed delivery times * 
all current robot locations, and the backtracking algorithm will need to search possibly all of them. 

The robot can have nC1 + nC2 + ... nCn = 2^n possibilities for current orders, and taking into account the different 
elapsed delivery times, the complexity becomes max HRTT of any order*2^n = O(2^n). If this problem is NP-hard, which 
  it likely is, then this is the best complexity we can get for a solution (assuming P != NP).
  

  
# Algorithm API

## `MultiorderSolver` class

This class represents an instantiable multiorder solver that takes in a weighted, undirected graph to 
solve batches of orders on. The user supplies the graph and the robot capacity in the constructor, and 
can repeatedly give the solver batches of orders to solve and find a satisfactory path for.

### `MultiorderSolver::calculateMultiorder` 

This function is the multiorder algorithm. It takes in a list of `Order`s and the robot's starting location 
and will output an ordered vector of `Move`s that the robot 
can do to deliver all the `Order`s on time and with the weight limitations of the robot, 
if possible. It also outputs all the intermediate "transits" the robot has to do, i.e. 
the entire path the robot has to take. If it's impossible, the algorithm returns an empty vector. 

### `Order` struct

An algo input specifying that order `id` will be `w` kg and go from node `S` to node `D` following 
the weighted edges ("time to travel across the edge") of the graph.
Contains a unique order ID, source node, destination node, and the weight of the order. 
The weight can be heavier than the capacity of the robot, but it will lead to the algorithm 
returning no solution. 

From the problem definition, the algorithm must be able to find a solution such that 
this order, from picking it up to dropping it off, will take no longer than 
2 * the minimum travel time from `S` to `D`, and the robot cannot take this order 
if its current remaining capacity is less than`w`. 

### `Move` struct

An algo output specifying a move that the robot takes in delivering the input 
orders, where the robot goes to `node` and does `action` for order `id`. 



# ROS Global Planner Node 

## `MultiorderNode` class

`MultiorderNode` is the global planner for the robot that runs on the ground station and processes incoming orders into 
moves in batches, sending the moves to the robot one-by-one. 

A user creates a graph and specifies a robot's starting location and capacity, 
and then the user can create a `MultiorderNode` to process orders online for a robot travelling on this graph. The 
`MultiorderNode` is a ROS node that takes in live orders and then sends waypoints to the robot to fulfill these 
orders on-time, if possible. It batches the orders since the algorithm can only run when the robot does not have 
any partially completed orders. 

If there's no path that meets the timing requirement then the node will throw an error. TODO This may be changed 
to an optimization problem later, in which case the node will output the best possible path and throw a warning.

### How it does planning for online orders in batches

`MultiorderNode` will have an internal list of waiting orders and planned moves for those waiting orders. The actual planning acts as a 2-stage batched planner, with the waiting orders list 
acting as a first stage and the current moves list acting as a second stage. 

Every time a new order comes in, the order will be added to the waiting orders list, and 
once all the currently planned moves are done, the node will calculate the best path for 
processing those waiting orders.

The order flow is like this:

Order goes to the `MultiorderNode` -> order sits in `waitingOrders_` until `plannedMoves_` 
is empty -> order is processed, gets planned out in `plannedMoves_` -> 
robot carries out the moves to pick up and deliver the order.

<b>Why do a batched planner when you can just recalculate with every new order?</b> 
The problem with replanning is that you can only change the current moves when the robot 
is not in the middle of the order, i.e. it's picked up an order already. The robot can 
have nested orders, and once it picks up an order the algorithm cannot recalculate with 
a half-finished order. 

Then, the only way to do this given that the algorithm only takes in whole orders 
is to replan once the current "batch" of orders are done, since we're guaranteed 
that the robot has no orders that it's working on when we recalculate with our 
waiting orders. 

### Flow of how the ground station and robot interact

Ground station initializes.

Gets the first order, calculates the moves, and sends it immediately to the robot. 
Robot starts going to the first pickup, edge by edge. As it goes, more orders can 
come in and join the waitingOrders_ list.

The robot arrives and sends its status to the ground station, and the ground station 
sends the next waypoint/move for the robot to go to. 

...this repeats until the batch of moves is exhausted. 

Then, upon the last move getting removed, the ground station takes all waiting orders 
and plans those out into moves. It repeats.

...this repeats until all orders are finished, and the robot does its last move 
and idles until more orders come in. 

### Input topic (from user)

`multiorder_alg::order` is a message type with fields for the 
order id, starting node (the restaurant), destination node, and the weight. 

### Input topic (from robot)

`multiorder_alg::robotStatus` is a message type that gets the robot's location. 

### Output topic (to robot)

`multiorder_alg::waypoint` is a message type that tells the robot the node to go to 
and the action to do (as a string). 


## File overview

### `src/multiorder_node/include/MultiorderSolver.hpp` 

contains the ROS API that has the multiorder algorithm.

### `src/multiorder_node/include/MultiorderNode.hpp` 

contains the ROS node that faciliates online order processing and robot planning.

### `src/multiorder_node/src/multiorder_alg_node.cpp` 
is the high-level ROS node that instantiates a `MultiorderNode` and tests it with a graph of CMU. 
It does solver tests for the algorithms and an involved ground station test. 

## To run

This algorithm is written in a standalone ROS node that doesn't subscribe or publish anything. 
It just runs some unit tests to make sure the algorithm works. 

Make sure you have ROS Melodic installed on your computer. 

Install it at http://wiki.ros.org/melodic/Installation/Ubuntu

To build:
```
catkin init; catkin clean; catkin build; source devel/<your shell>.sh
```

To run the tests:
```
roslaunch multiorderSim.launch
```
