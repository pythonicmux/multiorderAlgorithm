# Overview

This is a proof-of-concept for an optimization (?) algorithm 
I'm designing and implementing for 18-500 ECE Senior Capstone at Carnegie Mellon, Spring 2021. 
I'm on Team C9 (GrubTub). 

## The problem is as follows:

The robot has a total capacity of C, and is at (0,0) on a coordinate plane. We're supplied 
a function d:((x_1, y_1), (x_2, y_2)) => R+ U {0} that supplies us the time it takes to travel between any 
two points on this plane, in seconds.

The input is a list of orders {((x_1,y_1), (x_2, y_2), w, t), ...}. 

(x_1,y_1) is the starting coordinate.

(x_2, y_2) is the ending coordinate.

w is the weight of the order.

t is the maximum time this order can take.

The robot can only fit up to C capacity at once, and can deliver/pick up multiple deliveries 
at the same location, if they all fit.

The output must be an ordered list {((x,y), action, idx), ...}, where action is in 
{Pickup, Dropoff}. Pickup means the robot picks up the order idx which starts at (x,y), 
and Dropoff means the robot drops off the order idx which ends at (x,y). 

This list assumes the robot takes time to travel across the points corresponding to the 
function d, and that the actual actions take 0 time. 

A verified YES-instance would be a list s.t. each order is dropped off, and ((x,y), Dropoff, idx) 
happens before the dropoff time of idx.
