#include "pfc/drive_to_point.h"

#include "ros/ros.h"

int
main( int argc , char* argv[] ){

	DriveToPoint drive_to_point;

	ros::init( argc , argv , "pfc_node");
	ros::NodeHandle node_handle;

	bool following_command = true;

	
	// getting odom
	ros::Subscriber odom_subscriber = node_handle.subscribe( "odom" , 1 , &DriveToPoint::handle_odom , &drive_to_point );
	//getting path
	ros::Subscriber path_subscriber = node_handle.subscribe( "path" , 1 , &DriveToPoint::handle_path , &drive_to_point );

	ros::Subscriber traj_line_subscriber = node_handle.subscribe( "chosen_trajectory_line", 1 , &DriveToPoint::handle_trajectory_line , &drive_to_point );



	// the command publisher
	ros::Publisher command_publisher = node_handle.advertise< geometry_msgs::Twist >( "/mobile_base/commands/velocity" , 1 , true);

	drive_to_point.lookahead_publisher = node_handle.advertise< geometry_msgs::Point >( "lookahead" , 1 , true );


	double frequency = 20.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		if(following_command){
			drive_to_point.advance_path();
			drive_to_point.follow_traj();
		}
		else{
			drive_to_point.update_path_index();
			drive_to_point.update_command();
		}
		//drive_to_point.generate_projections( 0.0 , 0.1 );	
		command_publisher.publish( drive_to_point.command );
		// std::cout << "command(v,w):(" << drive_to_point.command.linear.x << "," << drive_to_point.command.linear.z << ")" << std::endl;
		ros::spinOnce();
		timer.sleep();
	}

	return 0;
}
