#ifndef MAPPER_H 
#define MAPPER_H

#include <iostream> 
#include <vector>

#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h" 
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h" 
#include "map_msgs/OccupancyGridUpdate.h"
#include "geometry_msgs/Polygon.h"
#include "sim/RoundObstacles2D.h"

class Mapper { 
public:
	Mapper( const double& discretization = 0.1, const unsigned int& numRows = 801, const unsigned int& numCols = 801 );
	virtual ~Mapper();
	void handleOdometry( const nav_msgs::Odometry::ConstPtr& msg );
	void handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg ); 
	void update( void );
	void shadow_map();
	// void handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg );
	void handle_obstacles( const sim::RoundObstacles2D::ConstPtr& msg );
	void add_obstacle( const double& x1, const double& x2 , const double& y1, const double& y2 );
	bool checkMap( const double& x, const double& y, const double& radius, const double& threshold ); // new function
	// void fill_line();
	inline map_msgs::OccupancyGridUpdate& map_msg( void ){ return _map; }; 
	sim::RoundObstacles2D _obstacles;
	ros::Publisher map_publisher;
protected:
	double _discretization; 
	std::vector< double > _xs; 
	std::vector< double > _ys; 
	double _l0;
	double _locc; 
	double _lfree;
	nav_msgs::Odometry _odometry;
	std::vector< sensor_msgs::LaserScan > _scans;
	map_msgs::OccupancyGridUpdate _map;
};

std::ostream& operator<<( std::ostream& out, const Mapper& other );
#endif /* MAPPER_H */
