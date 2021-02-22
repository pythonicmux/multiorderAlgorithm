# Overview

This is a proof-of-concept for an optimization (?) algorithm 
I'm designing and implementing for 18-500 ECE Senior Capstone at Carnegie Mellon, Spring 2021. 
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
