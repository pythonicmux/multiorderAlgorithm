# Overview

This is a proof-of-concept for an algorithm that will act as a global planner for an
autonomous delivery robot (currently this is just a constraint satisfaction problem 
until I find some kind of cost function to make it an optimization problem). 
I'm designing and implementing this for 18-500 ECE Senior Capstone at Carnegie Mellon, Spring 2021. 
I'm on Team C9 (GrubTub). 

## The problem is as follows:

The robot has a total capacity of C, and is at node S in an undirected, weighted graph. The 
weight of an edge corresponds with the time the robot takes to travel across the edge.

We also get a function d(S,D) that can compute the shortest distance of any path from S to D for any two 
nodes in the graph S and D.

The input is a list of orders {(id, (S,D), w), ...}. 

id is the unique identification string of the order. 

S is the starting node, D is the destination node.

w is the weight of the order.

The robot can only fit up to C capacity at once, and can deliver/pick up multiple deliveries 
at the same location, if they all fit.

<b> 
The output must be an ordered list {(S, action, id), ...},
</b> where action is in 
{Pickup, Dropoff}. Pickup means the robot picks up the order id which starts at (x,y), 
and Dropoff means the robot drops off the order id which ends at (x,y). 

For each order id, (S, Pickup, id) must be before (D, Dropoff, id) in the list.

<b> 
This output is correct if and only if, for each order (id, (S,D), w, t), 
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

## API overview

### `MultiOrderNode` constructor

A user creates a graph and specifies a robot's starting location and weight capacity (to fit orders in), 
and then the user can create a `MultiorderNode` to run the algorithm on the graph. 

### `MultiorderNode::calculateMultiorder` 

This function is the multiorder algorithm. It takes in a list of `Order`s and will output an 
ordered vector of `Move`s that the robot 
can do to deliver all the `Order`s on time and with the weight limitations of the robot, 
if possible. If it's impossible, the algorithm returns an empty vector.

## File overview

### `src/multiorder_node/include/MultiorderNode.hpp` 

contains the ROS internal node and API that faciliates the multiorder algorithm. 

### `src/multiorder_node/src/multiorder_alg_node.cpp` 
is the high-level ROS node that instantiates a `MultiorderNode` and tests it with a graph of CMU. 

## To run

This algorithm is written in a standalone ROS node that doesn't subscribe or publish anything. 
It just runs some unit tests to make sure the algorithm works. 

Make sure you have ROS Melodic installed on your computer. 

To build:
```
catkin clean; catkin build; source devel/<your shell>.sh
```

To run the tests:
```
roslaunch multiorderSim.launch
```
