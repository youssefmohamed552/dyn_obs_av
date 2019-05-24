#include <iostream>

#include "ros/ros.h"
#include "tcl/trajectory_planner.h"





int
main( int argc, char* argv[] ){
	
	TrajectoryPlanner tcl;


	ros::init( argc , argv , "tcl_node" );
	ros::NodeHandle node_handle;

	ros::Subscriber path_subscriber = node_handle.subscribe( "path" , 1, &TrajectoryPlanner::handle_path , &tcl );
	ros::Subscriber odom_subscriber = node_handle.subscribe( "odom" , 1, &TrajectoryPlanner::handle_odom , &tcl );
	ros::Subscriber map_subscriber = node_handle.subscribe( "map" , 1, &TrajectoryPlanner::handle_map , &tcl );

	ros::Publisher trajectory_publisher = node_handle.advertise< tcl::Trajectory >( "trajectory" , 1 , true );
	ros::Publisher obstacled_trajectory_publisher = node_handle.advertise< trajectory_msgs::MultiDOFJointTrajectory >( "obstacled_trajectory" , 1 , true );

	ros::Publisher trajectory_point_publisher = node_handle.advertise< tcl::TrajectoryLine >( "chosen_trajectory_line" , 1 , true );
	ros::Publisher goal_pt_publisher = node_handle.advertise< geometry_msgs::Point >( "goal_pt" , 1, true );
	ros::Publisher close_pt_publisher = node_handle.advertise< geometry_msgs::Point >( "close_pt" , 1, true );


	double frequency = 5.0;
	ros::Rate timer( frequency );
	while( ros::ok() ){
		tcl.generate_projection();
		// tcl.generate_projections( 0.0,0.5 );
		trajectory_publisher.publish( tcl.trajectory );
		// obstacled_trajectory_publisher.publish( tcl.obstacled_trajectory );
		tcl.update_way_pt();
		// std::cout << "close pt : " << tcl.close << " and the goal point is : " << tcl.goal_pt << std::endl;
		tcl.choose_trajectory();
		trajectory_point_publisher.publish( tcl.chosen_trajectory_line );
		if( !tcl.path.poses.empty() ){
			goal_pt_publisher.publish( tcl.path.poses[ tcl.goal_pt ].pose.position );
			close_pt_publisher.publish( tcl.path.poses[ tcl.close ].pose.position );
		}
		ros::spinOnce();
		timer.sleep();
	}

	return 0;
}
