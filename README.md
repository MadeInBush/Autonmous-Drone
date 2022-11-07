# Autonmous-Drone
Code for an Autonmous drone, using ROS, Python and C++

Given the GPS coordinates of the target location, this code will make the drone takeoff, orientate itself correctly, then fly autonmously to its target location. The control of the motors is done using Cpp while the reading of the onboard GPS sensor is done using python. The two scripts communicate with each other using the ROS framework.  

There are two packages:
  - readGPS, which does what it says on the tin...reads a GPS sensor that is plugged into ttyAMA0. A service has been setup called read_gps which when called will return the current GPS location.
  
  - control, which is used to control the drone. 

Currently the drone flys blind but am working on a avoidance system to be integrated into the code
