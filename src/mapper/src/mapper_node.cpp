#include <iostream>

#include "ros/ros.h"

#include "mapper/mapper.h"

#include "map_msgs/OccupancyGridUpdate.h"


int
main( int argc , char* argv[] ){
	Mapper mapper;

	ros::init( argc, argv ,"mapper_node");
	ros::NodeHandle node_handle;

	mapper.map_publisher = node_handle.advertise< map_msgs::OccupancyGridUpdate >( "map" , 1 , true );

	ros::Subscriber obstacle_subscriber = node_handle.subscribe( "obstacles" , 1 , &Mapper::handle_obstacles , &mapper );
	
	// mapper.add_obstacle( 1.0 , 3.0 , 1.0 , 4.0 );
	// mapper.fill_line();
	mapper.map_publisher.publish( mapper.map_msg() );
	ros::spin();
	return 0;
}

