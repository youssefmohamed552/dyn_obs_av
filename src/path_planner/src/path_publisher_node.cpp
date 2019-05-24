#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

geometry_msgs::Pose
make_pose( const double& x , const double& y ){
	geometry_msgs::Pose pose;
	pose.position.x = x;
	pose.position.y = y;
	return pose;
}

geometry_msgs::PoseStamped
make_pose_stamped( const double& x , const double& y ){
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = make_pose( x , y );
	return pose_stamped;
}

nav_msgs::Path
make_path(){
	nav_msgs::Path path;

	

	path.poses.push_back(make_pose_stamped(0.0 , 0.0));
	path.poses.push_back(make_pose_stamped(0.5 , 0.5));
	path.poses.push_back(make_pose_stamped(1.0 , 1.0));
	path.poses.push_back(make_pose_stamped(1.5 , 1.5));
	path.poses.push_back(make_pose_stamped(2.0 , 2.0));
	path.poses.push_back(make_pose_stamped(2.5 , 2.5));
	path.poses.push_back(make_pose_stamped(3.0 , 3.0));
	path.poses.push_back(make_pose_stamped(3.5 , 3.5));
	path.poses.push_back(make_pose_stamped(4.0 , 4.0));
	path.poses.push_back(make_pose_stamped(4.5 , 4.5));
	path.poses.push_back(make_pose_stamped(5.0 , 5.0));
	// path.poses.push_back(make_pose_stamped(0.0 , 0.0));
	// path.poses.push_back(make_pose_stamped(0.0 , 0.0));

	return path;
}

void print_path( const nav_msgs::Path& path ){
	for(unsigned int i = 0; i < path.poses.size(); i++){
		std::cout << "pose : " << i << " x: " << path.poses[i].pose.position.x << " , y: " << path.poses[i].pose.position.y << std::endl;
	}
}

int
main(int argc, char* argv[]){
	
	nav_msgs::Path path = make_path();

	ros::init( argc , argv , "path_publisher" );
	ros::NodeHandle node_handle;
	
	ros::Publisher path_publisher = node_handle.advertise< nav_msgs::Path >( "path" , 1 , true);
	sleep(5.0);

	std::cout << "publishing path" << std::endl;
	double frequency = 1.0;
	ros::Rate timer( frequency );
//	while(ros::ok()){
//		path_publisher.publish( path );
//
//		ros::spinOnce();
//		timer.sleep();
//	}

	path_publisher.publish( path );
	ros::spin();
	//std::cout << "path made with size : " << path.poses.size() << std::endl;

	//print_path(path);

//	ros::Rate timer;

//	timer.sleep( 10 );

	return 0;
}

