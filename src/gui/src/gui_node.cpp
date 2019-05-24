#include <QtGui/QApplication>

#include "gui/gui.h"

using namespace std;

int
main( int argc , char* argv[] ){
	QApplication app( argc , argv );
	ros::init( argc , argv , "gui" );
	ros::NodeHandle node_handle;
	GUI gui;
	ros::Subscriber subscriber_reset_odomotery = node_handle.subscribe( "laserscan" , 1 , &GUI::handleLaserScan , &gui );
	ros::Subscriber subscriber_odom = node_handle.subscribe( "odom" , 1 , &GUI::handleOdom , &gui );
	ros::Subscriber subscriber_goal = node_handle.subscribe( "goal" , 1 , &GUI::handleGoal , &gui );
	ros::Subscriber subscriber_lookahead = node_handle.subscribe( "lookahead" , 1, &GUI::handleLookahead , &gui );
	ros::Subscriber subscriber_path = node_handle.subscribe( "path" , 1 , &GUI::handlePath , &gui );
	ros::Subscriber subscriber_projection = node_handle.subscribe( "projection" , 1 , &GUI::handleProjection , &gui );

	ros::Subscriber substciber_trajectory = node_handle.subscribe( "trajectory" , 1 , &GUI::handleTrajectory , &gui );
	ros::Subscriber substciber_obstacled_trajectory = node_handle.subscribe( "obstacled_trajectory" , 1 , &GUI::handleObstacledTrajectory , &gui );
	ros::Subscriber substciber_chosen_trajectory_pint = node_handle.subscribe( "chosen_trajectory_line" , 1 , &GUI::handleChosenTrajectoryLine , &gui );
	//ros::Subscriber subscriber_obstacles = node_handle.subscribe( "obstacles" , 1 , &GUI::handleObstacles , &gui );
	ros::Subscriber subscriber_map = node_handle.subscribe( "map" , 1 , &GUI::handleMap , &gui );
	ros::Subscriber subscriber_goal_pt = node_handle.subscribe( "goal_pt" , 1, &GUI::handleGoalPt , &gui );
	ros::Subscriber subscriber_close_pt = node_handle.subscribe( "close_pt" , 1, &GUI::handleClosePt , &gui );
	ros::Subscriber subscriber_random_points = node_handle.subscribe( "random_points" , 1 , &GUI::handleRandomPoints , &gui );
	ros::Subscriber subscriber_rrt_start = node_handle.subscribe( "rrt_start" , 1 , &GUI::handleRRTStart , &gui );
	ros::Subscriber subscriber_rand_point = node_handle.subscribe( "new_rand" , 1 , &GUI::handleRandPoint , &gui );
	ros::Subscriber subscriber_near_point = node_handle.subscribe( "near_point" , 1 , &GUI::handleNearPoint , &gui );
	ros::Subscriber subscriber_new_point = node_handle.subscribe( "new_point" , 1 , &GUI::handleNewPoint , &gui );
	ros::Subscriber subscriber_projected_obstacles = node_handle.subscribe( "projected_obstacles" , 1 , &GUI::handleProjectedObstacles , &gui );

	gui.show();
	return app.exec();
}

