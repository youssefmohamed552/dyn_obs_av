#ifndef GUI_H
#define GUI_H

#include <iostream>
#include "ros/ros.h"

#include <QtOpenGL/QGLWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QTimer>
#include <QPushButton>
#include <QVBoxLayout>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "tcl/Trajectory.h"
#include "tcl/TrajectoryLine.h"
#include "path_planner/RRT.h"
#include "sim/RoundObstacles2D.h"
//#include "tcl/Projections.h"

class GUI: public QGLWidget {
	Q_OBJECT;
	public:
	GUI(QWidget * parent = NULL);
	virtual ~GUI();

	void handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg);
	void handleOdom( const nav_msgs::Odometry::ConstPtr& msg);
	void handleGoal( const geometry_msgs::Pose::ConstPtr& msg);
	void handlePath( const nav_msgs::Path::ConstPtr& msg);
	void handleProjection( const nav_msgs::Path::ConstPtr& msg);
	void handleLookahead( const geometry_msgs::Point::ConstPtr& msg);
	void handleTrajectory( const tcl::Trajectory::ConstPtr& msg );
	// void handleObstacles( const geometry_msgs::Polygon::ConstPtr& msg );
	void handleMap( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
	void handleObstacledTrajectory( const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg );
	void handleChosenTrajectoryLine( const tcl::TrajectoryLine::ConstPtr& msg );
	void handleGoalPt( const geometry_msgs::Point::ConstPtr& msg );
	void handleClosePt( const geometry_msgs::Point::ConstPtr& msg );
	void handleRandomPoints( const geometry_msgs::Polygon::ConstPtr& msg );
	void handleRRTStart( const path_planner::RRT::ConstPtr& msg );
	void handleRandPoint( const geometry_msgs::Point::ConstPtr& msg );
	void handleNearPoint( const geometry_msgs::Point::ConstPtr& msg );
	void handleNewPoint( const geometry_msgs::Point::ConstPtr& msg );
	void handleProjectedObstacles( const sim::RoundObstacles2D::ConstPtr& msg );

	protected slots:
		void timer_callback( void );

	protected:
	virtual void initializeGL();
	virtual void resizeGL(int width, int height);
	virtual void paintGL();
	void drawCoordinateSystem(void );
	void drawGrid();
	void drawLaserScan( const sensor_msgs::LaserScan& laserscan, const double& red = 0.0, const double& green = 0.0 , const double& blue = 0.0 );
	void drawRobot( const geometry_msgs::Pose& pose , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0, const double& radius = 0.1225);
	void drawPath( const nav_msgs::Path& path , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0 , const double& width = 1.0 );
	void drawTrajectory( const tcl::Trajectory& trajectory ,const double& red= 0.0 , const double& green = 0.0  , const double& blue = 0.0 , const double& width = 1.0 );
	void drawTrajectoryLine( const tcl::TrajectoryLine& trajectory_line , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0, const double& width = 1.0 );
	void drawTrajectoryPoint( const trajectory_msgs::MultiDOFJointTrajectoryPoint& trajectory_point , const double& red = 0.0 , const double& green = 0.0, const double& blue = 0.0 , const double& width = 1.0 );
	void drawPoint( const geometry_msgs::Point& point, const double& red = 0.0, const double& green = 0.0 , const double& blue = 0.0 , const double& size = 1.0);
void drawLine( const geometry_msgs::Point& p1 , const geometry_msgs::Point& p2 , const double& red , const double& green , const double& blue , const double& width );
	void drawPolygon( const geometry_msgs::Polygon& poly, const double& red = 0.0, const double& green = 0.0, const double& blue = 0.0, const double& width = 1.0 );
	void drawMap( const map_msgs::OccupancyGridUpdate& map, const double& red , const double& green  , const double& blue );
	void drawPoints( const geometry_msgs::Polygon& points , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0 , const double& size = 0.5 );
	void drawRRT( const path_planner::RRT& tree , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0 , const double& width = 1.0 );
	void drawObstacles( const sim::RoundObstacles2D& obs , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0 , const double& width = 1.0 );
	void drawObstacle( const sim::RoundObstacle2D& ob , const double& red = 0.0 , const double& green = 0.0 , const double& blue = 0.0 , const double& width = 1.0 );
	
	virtual void keyPressEvent(QKeyEvent* event);
	void handle_run_button();

	QTimer _timer;
	double _zoom;
	std::pair< double , double > _center;
	bool _odom_received;
	double occ;

	sensor_msgs::LaserScan _laserscan;
	nav_msgs::Odometry _odom;
	nav_msgs::Path _path;
	nav_msgs::Path _projection;
	geometry_msgs::Pose _goal;
	geometry_msgs::Point _lookahead;
	geometry_msgs::Point _goal_pt;
	geometry_msgs::Point _close_pt;
	geometry_msgs::Point _rand_point;
	geometry_msgs::Point _near_point;
	geometry_msgs::Point _new_point;
	tcl::Trajectory _trajectory;
	trajectory_msgs::MultiDOFJointTrajectory _obstacled_trajectory;
	tcl::TrajectoryLine _chosen_trajectory_line;
	geometry_msgs::Polygon _obstacle;
	map_msgs::OccupancyGridUpdate _map;
	geometry_msgs::Polygon _random_points;
	path_planner::RRT _rrt_start;
	sim::RoundObstacles2D _projected_obstacles;
	QVBoxLayout* main_layout;


	QPushButton* run_button;



};
#endif /* GUI_H */

