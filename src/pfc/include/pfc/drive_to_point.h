#include <iostream>
#include <vector>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "tcl/TrajectoryLine.h"
#include "ros/ros.h"


class DriveToPoint{
	public:
		DriveToPoint();
		virtual ~DriveToPoint();

		void handle_trajectory_line( const tcl::TrajectoryLine::ConstPtr& msg );
		void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
		void handle_path( const nav_msgs::Path::ConstPtr& msg );

		void update_path_index();
		void update_command();
		void advance_path();
		void follow_traj();
		// void adaptive_pure_pursuit();
		// nav_msgs::Path generate_a_trajectory( const double& direction_angle , const double& v, const double& k );

		unsigned int path_index;
		double K_p;
		double max_speed;
		double angle_threshold;
		double distance_threshold;
		double lookahead_distance;
		double number_of_projection_samples;

		// nav_msgs::Path trajectory;
		tcl::TrajectoryLine trajectory_line;
		nav_msgs::Odometry odom;
		geometry_msgs::Point lookahead;
		geometry_msgs::Twist command;
		geometry_msgs::Pose goal;
		nav_msgs::Path path;
		ros::Publisher lookahead_publisher;
		//std::vector< nav_msgs::Path > projection;



};
