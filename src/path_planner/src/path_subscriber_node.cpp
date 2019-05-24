#include "ros/ros.h"

#include "geometry_msgs/Point.h"
#include "nav_msgs/Path.h"

void
print_path( const nav_msgs::Path::ConstPtr& msg ){
	nav_msgs::Path path = *msg;
	for(unsigned int i = 0; i < path.poses.size(); i++){
		geometry_msgs::Point point = path.poses[i].pose.position;
		std::cout << "( " << point.x << " , " << point.y << " )" << std::endl;
	}
	return;
}

int
main( int argc , char* argv[] ){
	ros::init( argc , argv , "path_subscriber" );
	ros::NodeHandle node_handle;

	ros::Subscriber path_subscriber = node_handle.subscribe( "path" , 1 , &print_path );

	ros::spin();
	return 0;
}
