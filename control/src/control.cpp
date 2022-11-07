#include <control/control.hpp>

Control::Control(ros::NodeHandle *n){

    this->state_sub = n->subscribe<mavros_msgs::State>
            ("mavros/state", 10, &Control::state_cb, this);
    
    
    this->velocity_pub = n->advertise<geometry_msgs::Twist>
                ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    

    this->yaw_pub = n->advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 10);


    this->arming_client = n->serviceClient<mavros_msgs::CommandBool>
                ("mavros/cmd/arming");

    this->set_mode_client = n->serviceClient<mavros_msgs::SetMode>
                ("mavros/set_mode");

    this->takeoff_command = n->serviceClient<mavros_msgs::CommandTOL>
                ("/mavros/cmd/takeoff");

    this->land_command  = n->serviceClient<mavros_msgs::CommandTOL>
                ("/mavros/cmd/land");
    
	this->getGPS_service = n->serviceClient<readGPS::getCurrentGPS>
            ("get_gps");

	this->takeoff = false;

}

void Control::readGPS(float &longitude, float &latitude){

  //string delimiter = ",";
  
  
  readGPS::getCurrentGPSRequest req;
  readGPS::getCurrentGPSResponse res; 
  
  this->getGPS_service.call(req, res);
 
  ros::Duration(2).sleep();
  
  latitude = res.latitude;
  longitude = res.longitude;
  
  ROS_INFO("GPS: %f , %f", latitude, longitude);
//   int pos = res.gps.find(delimiter);


//   latitude = stod(res.gps.substr(0, res.gps.find(delimiter))); // breaking down the messaged into latitude and longitude 
//   res.gps.erase(0, pos + delimiter.length());
//   longitude = stod(res.gps.substr(0, res.gps.length()));

} 

void Control::findYaw(){ //finding the yaw for the drone to reach its target
	

	array<float, 4>lat_long = {0,0, this->target_location[0],this->target_location[1]};

 	readGPS(lat_long[0], lat_long[1]);
	

	float X , Y, bearing ,deltaA;

	convertingDegreesToRadians(lat_long);

	deltaA = lat_long[3] - lat_long[1]; //long2-long1


	X = cos(lat_long[2]) * sin(deltaA);


	Y = (cos(lat_long[0]) * sin(lat_long[2])) - (sin(lat_long[0]) * cos(lat_long[2]) * cos(deltaA));


	this->_yaw = atan2(X, Y);
	
}

void Control::convertingDegreesToRadians(array<float,4> &radian) { //converting between degrees and radians
	
	for (int i = 0; i < 4; i++) {
		radian[i] = (radian[i] * M_PI) / 180;
	}
	
}

void Control::orienate(){
	
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
		
	this->yaw_pub.publish(srv_target);
	ros::Duration(5).sleep();
	ROS_INFO("Published yaw");
	
}

void Control::land(){

	mavros_msgs::CommandTOL srv_land;
	
	srv_land.request.altitude = 0;

	while(ros::ok()){
			
		if(this->land_command.call(srv_land) && srv_land.response.success){
			ROS_INFO("landing");
			break;		
		}
		
	}

}

void Control::armAndTakeoff(){ 
     	//puts the pixhawk into guided mode and then arms it, then will take off to 2 metres
	
	while(ros::ok() && !this->current_state.connected){
        	ros::spinOnce();
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
	    if( this->current_state.mode != "GUIDED" && (ros::Time::now() - last_message > ros::Duration(5.0))){
				if( this->set_mode_client.call(guided_set_mode) && guided_set_mode.response.mode_sent){
					ROS_INFO("Guided enabled");
				}
				last_message = ros::Time::now();
		} else {
		    if( !this->current_state.armed && (ros::Time::now() - last_message > ros::Duration(5.0))){
		        if( this->arming_client.call(arm_cmd) && arm_cmd.response.success){
		            ROS_INFO("Vehicle armed");
					ros::Duration(4).sleep();
			    
					if(this->takeoff_command.call(srv_takeoff) && srv_takeoff.response.success){
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
		
	this->takeoff = check;
}

void Control::getTargetLocation(){
    cout << "Please enter the latituded of the destination: ";
	cin >> this->target_location[0];
	cout << "/nPlease enter the longitude of the destination: ";
	cin >> this->target_location[1];

    findYaw();

}

void Control::state_cb(const mavros_msgs::State::ConstPtr& msg){
    this->current_state = *msg;
}

void Control::move(){

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