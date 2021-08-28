# Autonmous-Drone
Code for an Autonmous drone, using mavros and C++

Given the GPS coordinates of the target location, this code will make the drone takeoff, orientate itself correctly, then fly autonmously to its target location. The control of the motors is done using Cpp while the reading of the onboard GPS sensor is done using python. The two scripts communicate with each other using the ROS framework.  

Currently the drone flys blind but am working on a avoidance system to be integrated into the code
