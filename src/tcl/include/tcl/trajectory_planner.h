#include <iostream>
#include <vector>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "std_msgs/Time.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "std_msgs/Int8.h"
#include "tcl/Trajectory.h"
#include "tcl/TrajectoryLine.h"



class TrajectoryPlanner{
	public:
		TrajectoryPlanner();
		virtual ~TrajectoryPlanner();

		void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
		void handle_path( const nav_msgs::Path::ConstPtr& msg );
		void handle_map( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );

		void choose_trajectory();
		double calculate_safety_of_traj( const int& index );
		double gen_time_limit( const nav_msgs::Path& p , const double& speed , const unsigned int& begin , const unsigned int& end );
		double is_point_occupied( const geometry_msgs::Pose2D& pose );
		void generate_projection();
		void trajectories_intersecting_with_obstacles();
		void update_way_pt();
		tcl::TrajectoryPoint sample_point( const tcl::TrajectoryPoint& prev_point , const double& direction_angle );
		tcl::TrajectoryLine generate_line( const double& direction_angle );

		// int calculate_safety_of_traj( const trajectory_msgs::MultiDOFJointTrajectoryPoint& traj_point );

		double lookahead_distance;
		double number_of_projection_samples;
		double max_speed;
		double discretization;
		double time_limit;
		double dt;
		double occ;
		double free_cell;
		int cost_threshold;
		int close;
		int goal_pt;
		bool odom_received;
		bool map_received;


		nav_msgs::Path path;
		nav_msgs::Odometry odom;
		// std::vector< nav_msgs::Path >projection;
		// nav_msgs::Path trajectory;
		// trajectory_msgs::MultiDOFJointTrajectory trajectory;
		trajectory_msgs::MultiDOFJointTrajectory obstacled_trajectory;
		// trajectory_msgs::MultiDOFJointTrajectoryPoint chosen_trajectory_point;
	  tcl::Trajectory trajectory;
	  tcl::TrajectoryLine chosen_trajectory_line;
		map_msgs::OccupancyGridUpdate map;

};
