#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int
main( int argc , char* argv[] ){
	ros::init( argc , argv , "command_pub" );
	ros::NodeHandle node_handle;

	geometry_msgs::Twist command;
	command.linear.x = 0.1;
	command.angular.z = 0.1;

	ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "/mobile_base/commands/velocity" , 1 , true );

	double frequency = 30.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		command_publisher.publish( command );
		ros::spinOnce();
	}
	
	return 0;
}

