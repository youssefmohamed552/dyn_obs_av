#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"

#include "pfc/drive_to_point.h"

double
calculate_distance( const geometry_msgs::Pose2D& p1 , const geometry_msgs::Pose2D& p2 ){
	double dist_x = p1.x - p2.x;
	double dist_y = p1.y - p2.y;
	return sqrt( dist_x * dist_x + dist_y * dist_y );
}

double
sgn( const double& arg ){
	if( arg < 0.0 ){
		return -1.0;
	} else {
		return 1.0;
	}
}


double
quaternion_to_yaw( geometry_msgs::Quaternion quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.x = cos( yaw/2.0 );
	quaternion.y = 0.0;
	quaternion.z = 0.0;
	quaternion.w = sin( yaw/2.0 );
	return quaternion;
}


geometry_msgs::Pose2D
pose_to_pose2d( const geometry_msgs::Pose robot_pose ){
	geometry_msgs::Pose2D pose_2d;
	pose_2d.x = robot_pose.position.x;
	pose_2d.y = robot_pose.position.y;
	pose_2d.theta = quaternion_to_yaw( robot_pose.orientation );
	return pose_2d;
}

geometry_msgs::PoseStamped
pose2d_to_pose_stamped( geometry_msgs::Pose2D pose2d ){
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose.position.x = pose2d.x;
	pose_stamped.pose.position.y = pose2d.y;
	pose_stamped.pose.orientation = yaw_to_quaternion( pose2d.theta );
	return pose_stamped;
}



geometry_msgs::Point
relative_position( const geometry_msgs::Pose2D& robot , const geometry_msgs::Pose2D world ){
	geometry_msgs::Point local;
	local.x = cos( robot.theta ) * ( world.x - robot.x ) + sin( robot.theta ) * ( world.y - robot.y );
	local.y = -sin( robot.theta ) * ( world.x - robot.x ) + cos( robot.theta ) * ( world.y - robot.y );
	return local;
}


DriveToPoint::
DriveToPoint()
	:path_index( 0 ),
	 K_p( 1.0 ),
	 max_speed( 0.25 ), 
	 angle_threshold( M_PI/ 8.0 ), 
	 distance_threshold( 0.1 ), 
	 lookahead_distance( 0.25 )
	 // number_of_projection_samples( 10 ),
	 //projection( std::vector< nav_msgs::Path >( number_of_projection_samples ) )
	 {}

DriveToPoint::
~DriveToPoint(){}


void
DriveToPoint::
handle_trajectory_line( const tcl::TrajectoryLine::ConstPtr& msg ){
	trajectory_line = *msg;
	path_index = 0;
	return;
}


void
DriveToPoint::
handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){
	// std::cout << "handling odom" << std::endl;
	odom= *msg;
	return;
}

void
print( const nav_msgs::Path& path ){
	std::cout << "path: ";
	for( unsigned int i = 0;i < path.poses.size(); i++ ){
		std::cout << "(" << path.poses[i].pose.position.x << "," << path.poses[i].pose.position.y << ")";
	}
	std::cout << std::endl;
}

void
DriveToPoint::
handle_path( const nav_msgs::Path::ConstPtr& msg ){
	std::cout << "path recieved with " << msg->poses.size() << " poses " << std::endl;
	path = *msg;
	print( path );
}


void
DriveToPoint::
advance_path(){
	if( trajectory_line.points.empty() ||  ( path_index >= trajectory_line.points.size() )) return;
	path_index++;
	return;
}


void
DriveToPoint::
update_path_index(){
	// if(path_index >= path.poses.size() ) return;
	geometry_msgs::Pose2D robot_pose = pose_to_pose2d( odom.pose.pose );

	if(!(path.poses.empty())){ // there exists a path
		double distance_to_path_index= calculate_distance( robot_pose , pose_to_pose2d( path.poses[ path_index ].pose ) );
		//update path index as long as it is within the lookahead area
		while( distance_to_path_index < lookahead_distance && path_index < path.poses.size()-1 ){
			// std::cout << " updating path index " << std::endl;
			path_index++;
			lookahead_publisher.publish( path.poses[ path_index ].pose.position );
			distance_to_path_index = calculate_distance( robot_pose , pose_to_pose2d( path.poses[ path_index ].pose ));
		}
		std::cout << "path index: (" << path_index << "):(" << path.poses[path_index].pose.position.x << "," << path.poses[path_index].pose.position.y << ") while size is : " << path.poses.size() <<  std::endl;
		//if( distance_to_path_index < distance_threshold ){
			//if( path_index >= path.poses.size() ){
				//std::cout << "path reset" << std::endl;
				//path.poses.clear();
				//path_index = 0;
				//return;
			//}
		//}

	}
	return;
}



void
DriveToPoint::
follow_traj(){
	if( trajectory_line.points.empty() ||  ( path_index >= trajectory_line.points.size() )){
		command.linear.x = 0.0;
		command.angular.z = 0.0;
		return;
	}
	command.linear.x = trajectory_line.points[ path_index ].linear;
	command.angular.z = trajectory_line.points[ path_index ].angular;
	return;
}



void
DriveToPoint::
update_command(){
	if(!path.poses.empty()){
		geometry_msgs::Pose2D robot_pose = pose_to_pose2d( odom.pose.pose);
		// geometry_msgs::Pose2D goal_pose = pose_to_pose2d( goal );
		geometry_msgs::Point relative_lookahead = relative_position( robot_pose , pose_to_pose2d( path.poses[ path_index].pose ) );
		double distance_to_goal = calculate_distance( robot_pose , pose_to_pose2d( path.poses[path_index].pose ) );
		double relative_angle = atan2( relative_lookahead.y , relative_lookahead.x );

		if( fabs( relative_angle ) < angle_threshold ){
			command.linear.x = command.linear.x + 0.1;
			if( command.linear.x > max_speed){
				command.linear.x = max_speed;
			}
			command.angular.z = 0.0;
		} else {
			command.angular.z = (sgn( relative_angle ))*0.5;
			command.linear.x = 0.0;
		}
	} else {
		command.linear.x = 0.0;
		command.angular.z = 0.0;
	}
	return;
}
