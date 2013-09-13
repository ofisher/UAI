This UAI is designed to provide a consistent interface for the path planner no matter which robot, or simulator is being driven.

The UAI accepts input in the following form

command0,time0,optional arguments:command1,time1, optional arguments:.....

E.g. 'home,5000:move,20000,1.5,-2.2,-1.65,-1.2,-0.7,-0.6' (without quotes, terminated by a new line)

The arm will move to the home position, and arrive there in 5 seconds
It will then move to [1.5,-2.2,-1.65,-1.2,-0.7,-0.6] and arrive there 20 seconds from when the command is sent (or 15 seconds after moving home)

Whether this coordinate is interpreted as joint angles (in radians) or Cartesian coordinates (in meters, defined from the base of the robot) depends on the mode of the UAI (i.e on command line arguments passed to UAI.py)

At any time a new set of coordinates can be sent to the UAI, this has the affect of completely overwriting the current set of coordinates. Current implementation is that if the arm is already moving it will keep moving on its current course until there is too much error between where it is headed, and where it should be headed, then it stops and moves to updated position.

Note that any times passed are in ms from the time they are passed, so an updated coordinate will have a different time associated to it.

Input is taken from standard in by default, or optionally the UAI can run a program which outputs the coordinates (for example a path planner). Eg --planner 'python Planner.py'

For a list of command line arguments use python UAI.py -h
