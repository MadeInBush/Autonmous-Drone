#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <mavros_msgs/Thrust.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>

#include <readGPS/getCurrentGPS.h>
#include <readGPS/getCurrentGPSRequest.h>
#include <readGPS/getCurrentGPSResponse.h>

/*#include <read_sensors/getGPS.h>
#include <read_sensors/getGPSRequest.h>
#include <read_sensors/getGPSResponse.h>*/

#include <iostream>
#include <math.h>
#include <array>
#include <string>
#include <iostream>

#define _USE_MATH_DEFINES

using namespace std;

class Control{

    public:
        Control(ros::NodeHandle *n);

        //callback functions
        void state_cb(const mavros_msgs::State::ConstPtr& msg);

        //reading gps functions
        void getTargetLocation();
        void convertingDegreesToRadians(array<float,4> &radian);
        void readGPS(float &latitude, float &longitude);
        void findYaw();

        //control functions    
        void orienate();
        void armAndTakeoff();
        void land();
        void move();
        

        //varaibles
        array<float, 2> target_location;
        bool takeoff;
        double _yaw;
    
    private:

        //subscribers
	    ros::Subscriber state_sub;

        //publishers
        ros::Publisher velocity_pub;
    	ros::Publisher yaw_pub;

        //services
        ros::ServiceClient arming_client;
        ros::ServiceClient set_mode_client;
        ros::ServiceClient takeoff_command;
        ros::ServiceClient land_command;    
        ros::ServiceClient getGPS_service;

        //messages
        mavros_msgs::State current_state;


};

