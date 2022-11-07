#include <control/control.hpp>

int main(int argc, char **argv){
    
    ros::init(argc, argv, "Jester_Control");
   	ros::NodeHandle n;
	ros::Rate rate(20);

    Control drone_control = Control(&n);

    drone_control.getTargetLocation();
	drone_control.findYaw();
	
	do{
		drone_control.armAndTakeoff();
	}while(!drone_control.takeoff);
	
	drone_control.orienate();
	//move();
	
	//land(land_command);


    return 0;

}