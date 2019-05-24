#include "sim/obstacles.h"
#include "ros/ros.h"


int
main( int argc , char* argv[] ){
	Obstacles obstacles;
	ros::init( argc , argv , "obstacles_publisher_node" );
	ros::NodeHandle node_handle;

	ros::Publisher obstacles_publisher = node_handle.advertise< sim::RoundObstacles2D >( "obstacles" , 1 , true );

	double frequency = 10.0;
	int i = 0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		std::cout << "publishing obstacles " << std::endl;
		obstacles_publisher.publish( obstacles.round_obstacles2d );
		if( i >= 10 ){
			i = 0;
			// obstacles.update();
		}
		// obstacles.step( 1.0 / frequency );
		i++;
		timer.sleep();
	}
	return 0;
}
