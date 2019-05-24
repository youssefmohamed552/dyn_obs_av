#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "tcl/trajectory_planner.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"




double
calculate_distance( geometry_msgs::Pose2D p1 , geometry_msgs::Pose2D p2 ){
	return ( sqrt( ( ( p1.x - p2.x ) * ( p1.x - p2.x ) ) + ( ( p1.y - p2.y ) * ( p1.y - p2.y ) ) ) );
}


geometry_msgs::Quaternion 
yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion; 
}


double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

double
sgn( const double& arg ){
	if( arg < 0.0 ) return -1.0;
	else return 1.0;
}

double
row_to_x( const map_msgs::OccupancyGridUpdate& map, const int& row, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.width ) - 1 ) / 2 + row ) );
}

double
col_to_y( const map_msgs::OccupancyGridUpdate& map, const int& col, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.height ) - 1 ) / 2 + col ) );
}

int
x_to_row( const map_msgs::OccupancyGridUpdate& map, const double& x, const double& discretization ){
	return round( x / discretization + ( double )( ( map.width - 1 ) / 2 ) );
}

int
y_to_col( const map_msgs::OccupancyGridUpdate& map, const double& y, const double& discretization ){
	return round( y / discretization + ( double )( ( map.height - 1 ) / 2 ) );}


geometry_msgs::Transform
pose_to_transform( const geometry_msgs::Pose& pose ){
	geometry_msgs::Transform transform;
	transform.translation.x = pose.position.x;
	transform.translation.y = pose.position.y;
	transform.translation.z = pose.position.z;
	transform.rotation = pose.orientation;
	return transform;
}

geometry_msgs::Pose
transform_to_pose( const geometry_msgs::Transform& transform ){
	geometry_msgs::Pose pose;
	pose.position.x = transform.translation.x;
	pose.position.y = transform.translation.y;
	pose.position.z = transform.translation.z;
	pose.orientation = transform.rotation;
	return pose;
}


geometry_msgs::Pose2D
transform_to_pose2d( const geometry_msgs::Transform& transform ){
	geometry_msgs::Pose2D pose2d;
	pose2d.x = transform.translation.x;
	pose2d.y = transform.translation.y;
	pose2d.theta = quaternion_to_yaw( transform.rotation );
	return pose2d;
}




geometry_msgs::PoseStamped
pose2d_to_pose_stamped( const geometry_msgs::Pose2D& pose ){
	geometry_msgs::PoseStamped pose_stamped; 
	pose_stamped.pose.position.x = pose.x; 
	pose_stamped.pose.position.y = pose.y; 
	pose_stamped.pose.orientation = yaw_to_quaternion( pose.theta ); 
	return pose_stamped;
}


geometry_msgs::Pose2D
pose_to_pose2d( const geometry_msgs::Pose pose ){
	geometry_msgs::Pose2D pose2d;
	pose2d.x = pose.position.x;
	pose2d.y = pose.position.y;
	pose2d.theta = quaternion_to_yaw( pose.orientation );
	return pose2d;
}


TrajectoryPlanner::
TrajectoryPlanner()
	: lookahead_distance( 2.5 ),
		number_of_projection_samples( 100 ),
		max_speed( 0.25 ),
		discretization( 0.1 ),
		time_limit( 6.0 ),
		dt( 1.0 / 20.0 ),
		occ( log( 0.9 / (1.0 - 0.9) ) ),
		free_cell( log( 0.1 / (1.0 - 0.1) ) ),
		cost_threshold( 0 ),
		close( 0 ),
		goal_pt( 0 ),
		odom_received( false ),
		map_received( false )
		{
		}

TrajectoryPlanner::
~TrajectoryPlanner(){
	}


double
TrajectoryPlanner::
gen_time_limit( const nav_msgs::Path& p , const double& curr_speed, const unsigned int& begin, const unsigned int& end ){
	if( end <= begin ) return 0.0;
	double time = 0.0;
	double speed = curr_speed;
	double dist_thresh = 0.1;
	unsigned int index = begin+1;
	geometry_msgs::Pose2D pose = pose_to_pose2d( p.poses[ begin ].pose );
	
	//TODO: fix heading
	double delta_y = p.poses[index].pose.position.y - pose.y;
	double delta_x = p.poses[index].pose.position.x - pose.x;
	double heading = atan2( delta_y , delta_x );
	while( index <= end ){
		double dist = calculate_distance( pose , pose_to_pose2d( p.poses[index].pose ) );
		// std::cout << "distance calculated " << dist << std::endl;
		if( dist < dist_thresh ){
			//TODO update threshold
			index++;
			delta_y = p.poses[index].pose.position.y - pose.y;
			delta_x = p.poses[index].pose.position.x - pose.x;
			heading = atan2( delta_y , delta_x );
		}
		if(speed >= max_speed)
			speed = max_speed;
		else
			speed = speed + 0.1;
		pose.x = pose.x + ( speed * dt * cos( heading ) );
		pose.y = pose.y + ( speed * dt * sin( heading ) );
		time = time + dt;

	}
	return time;
}

void
TrajectoryPlanner::
handle_path( const nav_msgs::Path::ConstPtr& msg ){
	// std::cout << "received a path of size : " << msg->poses.size() << std::endl;
	path = *msg;
	close = 0;
	goal_pt = 0;

	// goal = path.poses[ path.poses.size() - 1 ].pose;
	return;
}

void
TrajectoryPlanner::
handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
	//std::cout << "handling odom" << std::endl;
	// ROS_INFO("handling odom");
	odom_received = true;
	odom = *msg;
	return;
}


void
TrajectoryPlanner::
handle_map( const map_msgs::OccupancyGridUpdate::ConstPtr& msg ){
	// std::cout << "received a map \n \n\n\n\n\n " << std::endl;
	map_received = true;
	map = *msg;
	// std::cout << "handling map with width: " << map.info.width << " and height: " << map.info.height << " and resolution: " << map.info.resolution << std::endl;
	return;
}


void
TrajectoryPlanner::
update_way_pt(){
	if( path.poses.empty() ) return;
	if( close >= path.poses.size() -1 ) return;
	geometry_msgs::Pose2D robot_pose = pose_to_pose2d( odom.pose.pose );
	int way_pt = close;
	double min_distance = HUGE_VAL;
	double error_distance = calculate_distance( robot_pose , pose_to_pose2d( path.poses[way_pt].pose ) );
	// std::cout << "way_pt: " << way_pt << " , error_distance: " << error_distance << std::endl;
	while( way_pt < path.poses.size() && error_distance < min_distance ){
		min_distance = error_distance;
		// std::cout << "error_distance : " << error_distance << std::endl;
		way_pt++;
		if( way_pt >= path.poses.size() ) break;
		error_distance = calculate_distance( robot_pose , pose_to_pose2d( path.poses[way_pt].pose ) );
		if( error_distance < min_distance ) close = way_pt;
	}

	way_pt = close;
	double nonadaptive_lookahead_distance = 0; 
	while( way_pt < path.poses.size() && nonadaptive_lookahead_distance < lookahead_distance ){
		nonadaptive_lookahead_distance = calculate_distance( pose_to_pose2d( path.poses[close].pose ) , pose_to_pose2d( path.poses[way_pt].pose ) );	
		if( nonadaptive_lookahead_distance < lookahead_distance ) goal_pt = way_pt;
		way_pt++;
	}
	//TODO: fix assumptions about the length of the path and the current speed
	time_limit = gen_time_limit( path , odom.twist.twist.linear.x , close , goal_pt );

	
	/*
	std::cout << "updating the way_point" << std::endl;
	if( path.poses.empty() ) return;
	if( goal_pt >= path.poses.size() -1 ) return;
	geometry_msgs::Pose2D robot_pose = pose_to_pose2d( odom.pose.pose );
	int way_pt = close;
	geometry_msgs::Pose2D goal_pose = pose_to_pose2d( path.poses[ way_pt ].pose );
	double min = HUGE_VAL;
	double error_distance = calculate_distance( robot_pose , pose_to_pose2d( path.poses[ way_pt ].pose ) );
	double non_adaptive_lookahead = calculate_distance( pose_to_pose2d( path.poses[ close ].pose ) , goal_pose );
	while(way_pt < path.poses.size() &&  non_adaptive_lookahead < error_distance + lookahead_distance ){
		std::cout << "updating close point " << std::endl;
		goal_pose.x = path.poses[ way_pt ].pose.position.x;
		goal_pose.y = path.poses[ way_pt ].pose.position.y;
		if( error_distance < min ){
			close = way_pt;
			min = error_distance;
		}
		way_pt++;
		error_distance = calculate_distance( robot_pose , pose_to_pose2d( path.poses[ way_pt ].pose ) );
		non_adaptive_lookahead = calculate_distance( pose_to_pose2d( path.poses[ close ].pose ) , goal_pose );
	}

	way_pt = close;
	double adaptive_lookahead = calculate_distance( robot_pose , goal_pose );
	while( adaptive_lookahead < error_distance + lookahead_distance ){
		std::cout << "updating goal point " << std::endl;
		goal_pose.x = path.poses[ way_pt ].pose.position.x;
		goal_pose.y = path.poses[ way_pt ].pose.position.y;
		goal_pt = way_pt;
		adaptive_lookahead = calculate_distance( robot_pose , goal_pose );
		error_distance = calculate_distance( robot_pose , pose_to_pose2d( path.poses[ way_pt ].pose ) );
	}
	*/
		
}

tcl::TrajectoryPoint
TrajectoryPlanner::
sample_point( const tcl::TrajectoryPoint& prev_point , const double& direction_angle ){
	// std::cout << "sample point from pose[x,y,sig] : [" << pose.translation.x <<"," << pose.translation.y << "," << quaternion_to_yaw(pose.rotation) << "]" <<  std::endl;
	tcl::TrajectoryPoint point;
	

	point.pose.theta = prev_point.pose.theta + ( prev_point.angular * dt );
	point.pose.x = prev_point.pose.x + ( prev_point.linear * dt * cos( point.pose.theta ) ); 
	point.pose.y = prev_point.pose.y + ( prev_point.linear * dt * sin( point.pose.theta ) ); 
	if( prev_point.linear >= max_speed )
		point.linear = max_speed;
	else
		point.linear = prev_point.linear + 0.1;

	double k = ( direction_angle / point.linear );
	point.angular = k * point.linear;
	point.stamp = prev_point.stamp + dt;
	point.seq = prev_point.seq + 1;
	point.cost = is_point_occupied( point.pose );
	// std::cout << "(x,y,theta):(" << point.pose.x << "," << point.pose.y << "," << point.pose.theta <<  "), v: " << point.linear << ", w: " << point.angular << ", time: " << point.stamp << ",cost is : " << point.cost << " with occupied : " << occ << std::endl;

	return point;
}

tcl::TrajectoryLine
TrajectoryPlanner::
generate_line( const double& direction_angle ){
	// std::cout << "generate trajectory_line : " << direction_angle << std::endl;
	// std::cout << "generate a trajectory at an angle : " << direction_angle << " and velocity : " << v << " and curvature : " << k << std::endl;
	tcl::TrajectoryLine trajectory_line;

	// time track
	double time_stamp = 0.0;

	// calculating the the robot pose
	geometry_msgs::Pose2D robot_pose = pose_to_pose2d( odom.pose.pose );
	double v = odom.twist.twist.linear.x;
	if( !odom_received ){
		// std::cout << "no odom in system" << std::endl;
		robot_pose.x = 0;
		robot_pose.y = 0;
		robot_pose.theta = 0;
		v = 0.0000000000001;
	}

	double k = ( direction_angle / v );

	// make the first point at the robot center
	tcl::TrajectoryPoint point;
	point.pose = robot_pose;
	point.linear = v;
	point.angular = k * v;
	point.stamp = time_stamp;
	point.seq = 0;
	point.cost = 0.0;
	trajectory_line.points.push_back( point );
	bool found_obst = false;

	

	
	//sample points on a trajectory
	while( trajectory_line.points.back().stamp < time_limit ){
		// std::cout << "generating the " << i << " th point" << std::endl;
		point = sample_point( point , direction_angle );
		trajectory_line.points.push_back( point );
		// if( !found_obst && point.cost < occ ){
		trajectory_line.cost = trajectory_line.cost + fabs( point.cost );
		// 	continue;
		// }
		// found_obst = true;
		// trajectory_line.cost = trajectory_line.cost + occ;

	}
	if( !path.poses.empty()){
		trajectory_line.cost = trajectory_line.cost + calculate_distance( trajectory_line.points.back().pose , pose_to_pose2d( path.poses[goal_pt].pose) ); 
	}


	return trajectory_line;
}

void
TrajectoryPlanner::
generate_projection(){
	//std::cout << "gererate projection" << std::endl;
	// the change in angle between each projection
	// ROS_INFO("generating traj");

	trajectory.lines.clear();
	if( !path.poses.empty() && close >= path.poses.size() -1 ) {
		chosen_trajectory_line.points.clear();
		return;
	}

	double delta_alpha = ( M_PI / 3 ) / number_of_projection_samples;
	double angle = - M_PI/6; 
	
	// double angle = 0.0;
	for( unsigned int i = 0 ; i <= number_of_projection_samples; i++ ){
		// generate one of the sample lines
		// std::cout << "generating the " << i << " th trajectory" << std::endl;
		tcl::TrajectoryLine traj_line = generate_line( angle ); 
		angle = angle + delta_alpha;
		trajectory.lines.push_back( traj_line );
	}
	return;
}

double
TrajectoryPlanner::
is_point_occupied( const geometry_msgs::Pose2D& pose ){
	// std::cout << "checking if the cell is occupid" << std::endl;
	// if( map ) return 0;
	int row = x_to_row( map , pose.x , discretization );
	int col = y_to_col( map , pose.y , discretization );	
	// std::cout << "cell (x,y) : (" << cell_x << "," << cell_y << "):" <<   cell_y * map.width << std::endl;
	if( !map_received ) return 0.0;
	if( map.data[ ( row * map.width ) + col ] > free_cell ) return 1.0;
	else return 0.0;
}

double
TrajectoryPlanner::
calculate_safety_of_traj( const int& index ){
	// std::cout << "calculating the safty of trajectory point with index : " << index <<  std::endl;
	if( trajectory.lines[index].points.empty() ) return 0;
	double safety_level = HUGE_VAL;
	for(unsigned int i = 0; i < trajectory.lines[ index ].points.size(); i++){
		safety_level -= trajectory.lines[ index ].points[ i ].cost;
		// if(is_point_occupied( trajectory.points[ index ].transforms[i] ) == 1 || is_point_occupied( trajectory.points[ index ].transforms[i] ) == -1) break;
		// safety_total ++;
	}
	return safety_level;
}

void
TrajectoryPlanner::
trajectories_intersecting_with_obstacles(){
	/*
	if(trajectory.points.empty() ) return;
	for( unsigned int i = 0; i < trajectory.points.size(); i++ ){
		for( unsigned int j = 0; j < trajectory.points[i].transforms.size();j++){
			if( is_point_occupied(trajectory.points[i].transforms[j]) > 50 ){
				obstacled_trajectory.points.push_back( trajectory.points[i] );
				break;
			}
		}
	}
	*/
	return;
}


void
TrajectoryPlanner::
choose_trajectory(){
	// std::cout << "choosing a trajectory" << std::endl;
	if( trajectory.lines.empty() || path.poses.empty() ) return;
	/*
	geometry_msgs::Pose2D robot_pose = pose_to_pose2d( odom.pose.pose );
	// choose a starting trajectory point top check for safety
	double min_distance = HUGE_VAL;
	double desired_angle = M_PI;
	int prefered_traj_point = -1;
	
	geometry_msgs::Pose2D next_step_path = pose_to_pose2d( path.poses[ goal_pt ].pose );
	for( unsigned int i = 0; i < trajectory.lines.size() ; i++){
		double distance = calculate_distance( trajectory.lines[i].points.back().pose , next_step_path ); 
		if( distance < min_distance ){
			min_distance = distance;
			prefered_traj_point = i;
		}
	}

	// compute safety in the prefered one first
	int traj_index = prefered_traj_point;
	double cost_traj = trajectory.lines[ traj_index ].cost;
	int left_traj = prefered_traj_point -1;
	int right_traj = prefered_traj_point +1;
	*/

	// if( trajectory.lines[ traj_index ].cost > 0 ){
	int traj_index = 0;
	double cost_traj = trajectory.lines[ traj_index ].cost;
		// std::cout << "find min" << std::endl;
		for( int i = 1 ; i < trajectory.lines.size(); i++ ){
			// std::cout << "trajectory_line[" << i << "] has cost: " << trajectory.lines[i].cost << std::endl;
			if( trajectory.lines[i].cost < cost_traj ){
				cost_traj = trajectory.lines[i].cost;
				traj_index = i;
			}
		}
	// }




	
	/*
	while( cost_traj > 0 ){
		bool left = false;
		bool right = false;
		double left_cost = HUGE_VAL;
		double right_cost = HUGE_VAL;
		
		if( left_traj >= 0 ){
			left_cost = trajectory.lines[ left_traj ].cost;
			left_traj--;
		}
		if( right_traj < trajectory.lines.size() ){
			right_cost = trajectory.lines[ right_traj ].cost;
			right_traj++;
		}
		if( right_cost < left_cost && right_cost < cost_traj ){
			right = true;
			cost_traj = right_cost;
			traj_index = right_traj;
		}
		else if( left_cost < cost_traj ){
			left = true;
			cost_traj = left_cost;
			traj_index = left_cost;
		}
		else{
			left = false;
			right = false;
		}

		if( !left && !right ) break;
	}
	*/


	/*
	while( safety_of_traj < cost_threshold){
		std::cout << "looking for alternative trajectory_points" << std::endl;
		bool left = false;
		bool right = false;
		if( left_traj >= 0){
			left = true;
			cost_traj = trajectory.lines[ left_traj ].cost;
			if( cost_traj > cost_threshold){
				max_safety = cost_traj;
				traj_index = left_traj;
			}
			left_traj--;
		}
		if( right_traj < trajectory.lines.size() ){
			right = true;
			cost_traj = trajectory.lines[ right_traj ].cost;
			if( cost_traj > cost_threshold ){
				max_safety = cost_traj;
				traj_index = right_traj;
			}
			right_traj++;
		}

		if( !left && !right) break;

	}
	*/

	chosen_trajectory_line = trajectory.lines[ traj_index ];
	return;

}


