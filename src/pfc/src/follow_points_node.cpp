#include "pfc/drive_to_point.h"

#include "ros/ros.h"

int
main( int argc , char* argv[] ){

	DriveToPoint drive_to_point;

	ros::init( argc , argv , "pfc_node");
	ros::NodeHandle node_handle;

	// getting trajectory_line
	ros::Subscriber trajectory_line_subscriber = node_handle.subscribe( "chosen_trajectory_line" , 1 , &DriveToPoint::handle_trajectory_line , &drive_to_point );
	
	// getting odom
	ros::Subscriber odom_subscriber = node_handle.subscribe( "odom" , 1 , &DriveToPoint::handle_odom , &drive_to_point );

	// the command publisher
	ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "/mobile_base/commands/velocity" , 1 , true);

	double frequency = 20.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		drive_to_point.advance_path();
		drive_to_point.follow_traj();
		command_publisher.publish( drive_to_point.command );
		std::cout << "command(v,w):(" << drive_to_point.command.linear.x << "," << drive_to_point.command.linear.z << ")" << std::endl;
		ros::spinOnce();
		timer.sleep();
	}

	return 0;
}
