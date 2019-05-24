#include <iostream>
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"


int
main( int argc , char* argv[] ){
	ros::init( argc , argv , "odom_publisher" );
	ros::NodeHandle node_handle;

	nav_msgs::Odometry odom;
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	ros::Publisher odom_publisher = node_handle.advertise< nav_msgs::Odometry >( "odom" , 1 , true );

	odom_publisher.publish( odom );

	ros::spin();

	return 0;
}
