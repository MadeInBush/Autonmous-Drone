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

#include <read_sensors/getGPS.h>
#include <read_sensors/getGPSRequest.h>
#include <read_sensors/getGPSResponse.h>

#include <iostream>
#include <math.h>
#include <array>
#include <string>
#include <iostream>

#define _USE_MATH_DEFINES

using namespace std;

mavros_msgs::State current_state;



void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}


void readGPS(double &longitude, double &latitude, ros::ServiceClient get_GPS){

  string delimiter = ",";
  
  
  read_sensors::getGPSRequest req;
  read_sensors::getGPSResponse res; 
  
  get_GPS.call(req, res);
 
  ros::Duration(2).sleep();
  
  int pos = res.gps.find(delimiter);

  latitude = stod(res.gps.substr(0, res.gps.find(delimiter))); // breaking down the messaged into latitude and longitude 
  res.gps.erase(0, pos + delimiter.length());
  longitude = stod(res.gps.substr(0, res.gps.length()));


} 


void convertingDegreesToRadians(array<double,4> &radian) { //converting between degrees and radians
	
	for (int i = 0; i < 4; i++) {
		radian[i] = (radian[i] * M_PI) / 180;
	}
	
}


double findYaw(ros::ServiceClient get_GPS, array<double, 2> target_location){ //finding the yaw for the drone to reach its target
	

	array<double, 4>lat_long = {0,0, target_location[0],target_location[1]};

 	readGPS(lat_long[0], lat_long[1], get_GPS);
	

	double X , Y, bearing ,deltaA;

	convertingDegreesToRadians(lat_long);

	deltaA = lat_long[3] - lat_long[1]; //long2-long1


	X = cos(lat_long[2]) * sin(deltaA);


	Y = (cos(lat_long[0]) * sin(lat_long[2])) - (sin(lat_long[0]) * cos(lat_long[2]) * cos(deltaA));


	bearing = atan2(X, Y);

	return bearing;
	
}

bool armAndTakeoff(ros::Subscriber state, ros::ServiceClient arming, ros::ServiceClient set_mode, ros::Rate rate, ros::ServiceClient takeoff){ 
     	//puts the pixhawk into guided mode and then arms it, then will take off to 2 metres
	
	while(ros::ok() && !current_state.connected){
        	ros::spinOnce();
        	rate.sleep();
    }
    
    bool check = false;
	
	ROS_INFO("Connected");//mavros and the pixhawk are connected
			
	mavros_msgs::SetMode guided_set_mode;
    guided_set_mode.request.custom_mode = "GUIDED"; //setting the pixhawk to be in guided mode

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
	

	mavros_msgs::CommandTOL srv_takeoff;
	srv_takeoff.request.altitude = 2;

	ros::Time last_message = ros::Time::now();
	
	while(ros::ok()){
	    if( current_state.mode != "GUIDED" && (ros::Time::now() - last_message > ros::Duration(5.0))){
				if( set_mode.call(guided_set_mode) && guided_set_mode.response.mode_sent){
					ROS_INFO("Guided enabled");
				}
				last_message = ros::Time::now();
		} else {
		    if( !current_state.armed && (ros::Time::now() - last_message > ros::Duration(5.0))){
		        if( arming.call(arm_cmd) && arm_cmd.response.success){
		            ROS_INFO("Vehicle armed");
					ros::Duration(4).sleep();
			    
					if(takeoff.call(srv_takeoff) && srv_takeoff.response.success){
						ROS_INFO("Taking off");
						ros::Duration(10).sleep();
						check = true;
						break;
					}	 
		        }
		        last_message = ros::Time::now();
		    }
		}
	}
		
	return check;
}

void orienate(ros::Publisher yaw_pub){
	
	mavros_msgs::PositionTarget srv_target;
	
	srv_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_OFFSET_NED;

	srv_target.type_mask = 	     mavros_msgs::PositionTarget::IGNORE_AFX | 
                                     mavros_msgs::PositionTarget::IGNORE_AFY | 
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_VX  | 
                                     mavros_msgs::PositionTarget::IGNORE_VY  | 
                                     mavros_msgs::PositionTarget::IGNORE_VZ;

	srv_target.yaw = 1.5;
	srv_target.yaw_rate = 0.1; // kept slow
		
	yaw_pub.publish(srv_target);
	ros::Duration(5).sleep();
	ROS_INFO("Published yaw");
	
}


void move(ros::Publisher velocity_pub){

	ROS_INFO("Moving");
	geometry_msgs::Twist srv_velocity;
	
		
	ros::Duration(2).sleep();
	
	
	int current_location = 10, target_location = 20;
	
	while(current_location != target_location){
			ROS_INFO("Current: %d, target: %d", current_location, target_location);
			if((target_location - current_location) < 5 || (current_location - target_location) > -5){//drone is approaching final destination so will be slowed down ready for landing
					ROS_INFO("Chaning speed");
					if((target_location - current_location) < 1 || (current_location - target_location) > -1){//drone is ready to land	
							ROS_INFO("stopping");
							
							srv_velocity.linear.y = 0;
							srv_velocity.angular.y = 0;
							srv_velocity.linear.x = 0;
							srv_velocity.angular.x = 0;
							
							velocity_pub.publish(srv_velocity);
							break;			
						}else{
								ROS_INFO("Slowing down");
								
								srv_velocity.linear.y = 1 * cos(-1.5) ; //needs to take it in radians
								srv_velocity.angular.y = 1 * sin(1);
								srv_velocity.linear.x = 1 * sin(-1.5);
								srv_velocity.angular.x = 1 * cos(1);
							
								velocity_pub.publish(srv_velocity);
								
							}
					
				}else{
					
					srv_velocity.linear.y = 2.5 * cos(-1.5); //drone is not near landing site and so will just continue/start going at its normal speed
					srv_velocity.angular.y = 2.5 * sin(1);
					srv_velocity.linear.x = 2.5 * sin(-1.5);
					srv_velocity.angular.x = 2.5 * cos(1);
							
					velocity_pub.publish(srv_velocity);
					
					}
			
			ros::Duration(1).sleep();
			target_location--;
		
		}
	
	
	
}

void land(ros::ServiceClient land_command){

	mavros_msgs::CommandTOL srv_land;
	
	srv_land.request.altitude = 0;

	while(ros::ok()){
			
		if(land_command.call(srv_land) && srv_land.response.success){
			ROS_INFO("landing");
			break;		
		}
		
	}

}

void getTargetLocation(array<double, 2> &targetCoordinates){
	
	cout << "Please enter the latituded of the destination: ";
	cin >> targetCoordinates[0];
	cout << "/nPlease enter the longitude of the destination: ";
	cin >> targetCoordinates[1];	
	
}


int main(int argc, char **argv){

	ros::init(argc, argv, "Jester_Control");
   	ros::NodeHandle n;

	ros::Rate rate(20);
	
	//varibales
	array<double, 2> target_location;
	bool takeoff;
	double yaw_;
	
	ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = n.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = n.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient takeoff_command = n.serviceClient<mavros_msgs::CommandTOL>
			("/mavros/cmd/takeoff");
	ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>
			("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
	ros::ServiceClient land_command = n.serviceClient<mavros_msgs::CommandTOL>
			("/mavros/cmd/land");
	ros::Publisher yaw_pub = n.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);
    ros::ServiceClient getGPS_service = n.serviceClient<read_sensors::getGPS>
	    ("get_gps");
	
	
	getTargetLocation(target_location);
	yaw_ = findYaw(getGPS_service, target_location);
	
	
	do{
		takeoff = armAndTakeoff(state_sub, arming_client, set_mode_client, rate, takeoff_command);
	}while(!takeoff);
	
	orienate(yaw_pub, yaw_);
	move(velocity_pub);
	
	land(land_command);
	
	
	return 0;
	 
	
}
