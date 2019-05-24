#include "ros/ros.h"
#include "geometry_msgs/Pose.h"


int
main( int argc , char* argv[] ){
	ros::init( argc , argv , "executive_node" );
	ros::NodeHandle node_handle;

	geometry_msgs::Pose goal;
	goal.position.x = 5.0;
	goal.position.y = 3.0;
	goal.position.z = 0.0;
	goal.orientation.w = 1.0;
	goal.orientation.x = 0.0;
	goal.orientation.y = 0.0;
	goal.orientation.z = 0.0;
	ros::Publisher goal_publisher = node_handle.advertise< geometry_msgs::Pose >( "goal" , 1 , true );

	sleep( 3.0 );
	goal_publisher.publish( goal );
	ros::spin();
	return 0;
}
