#include "path_planner/rrt_path_planner.h"


int
main( int argc, char* argv[] )
{
	RRTPathPlanner rrt_planner;
	ros::init( argc , argv , "rrt_path_planner" );
	//set the debugging level manually to DEBUG
	// ROSCONSOLE_AUTOINIT;
	// log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger( ROSCONSOLE_DEFAULT_NAME );
	// my_logger->setLevel( ros::console::g_level_lookup[ros::console::levels::Debug] );
	ros::NodeHandle node_handle;

	ros::Subscriber odom_subscriber = node_handle.subscribe( "odom" , 1 , &RRTPathPlanner::handle_odom , &rrt_planner );
	ros::Subscriber goal_subscriber = node_handle.subscribe( "goal" , 1 , &RRTPathPlanner::handle_goal , &rrt_planner );
	ros::Subscriber map_subscriber = node_handle.subscribe( "map" , 1 , &RRTPathPlanner::handle_map , &rrt_planner );
	ros::Subscriber obstacle_subsciber = node_handle.subscribe( "obstacles" , 1 , &RRTPathPlanner::handle_obstacles , &rrt_planner );

	rrt_planner.rrt_publisher = node_handle.advertise< path_planner::RRT >( "rrt_start" , 1, true );
	rrt_planner.new_rand_publisher = node_handle.advertise< geometry_msgs::Point >( "new_rand" , 1, true );
	rrt_planner.near_point_publisher = node_handle.advertise< geometry_msgs::Point >( "near_point" , 1, true );
	rrt_planner.new_point_publisher = node_handle.advertise< geometry_msgs::Point >( "new_point" , 1, true );
	rrt_planner.path_publisher = node_handle.advertise< nav_msgs::Path >( "path" , 1, true );
	rrt_planner.projected_obstacle_publisher = node_handle.advertise< sim::RoundObstacles2D >( "projected_obstacles" , 1 , true );
	// rrt_planner.random_points_publisher = node_handle.advertise< geometry_msgs::Polygon >( "random_points" , 1 , true );
	ros::spin();
	return 0;
}
