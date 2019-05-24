#include <iostream>

#include <GL/gl.h>
#include <GL/glu.h>

#include "gui/gui.h"


double 
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

geometry_msgs::Quaternion yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion; 
}

GUI::
GUI( QWidget * parent ) : 
	QGLWidget ( parent ), 
	_timer(), 
	_zoom( 5.0 ), 
	_center( 0.5 , 0.5 ), 
	_odom_received( false ), 
	occ( log( 0.9 / ( 1.0 - 0.9 ) ) ),
	_laserscan(), 
	_odom(), 
	_goal(), 
	_path(),
	run_button(new QPushButton("RUN!!", this)),
	main_layout(new QVBoxLayout() )
	{
		setMinimumSize( 600 , 600 );
		setFocusPolicy( Qt::StrongFocus );
		connect( &_timer, SIGNAL( timeout() ), this, SLOT( timer_callback() ) );
		// UI program
		// createHorizontalGroupBox();
		// connect( run_button , SIGNAL( pressed() ) , this , SLOT( handle_run_button() ) );
		_timer.start( 10 );
		// run_button->show();
	}

GUI::
~GUI(){

}

void
GUI::
handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg ){
	std::cout << "in handleLaserScan" << std::endl;
	_laserscan = *msg;
	updateGL();
	return;
}

void
GUI::
handleOdom(const nav_msgs::Odometry::ConstPtr& msg ){
	std::cout << "in handleOdom" << std::endl;
	_odom_received = true;
	_odom = *msg;
	updateGL();
	return;
}

void
GUI::
handleGoal( const geometry_msgs::Pose::ConstPtr& msg ){
	std::cout << "in handleGoal" << std::endl;
	_goal = *msg;
	updateGL();
	return;
}

void
GUI::
handlePath( const nav_msgs::Path::ConstPtr& msg ){
	std::cout << "in handlePath" << msg->poses.size() << std::endl;
	_path = *msg;
	updateGL();
	return;
}

void
GUI::
handleTrajectory( const tcl::Trajectory::ConstPtr& msg ){
	std::cout << " in handleTrajectory with size: " << msg->lines.size() <<  std::endl;
	_trajectory = *msg;
	updateGL();
	return;
}

void
GUI::
handleObstacledTrajectory( const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg ){
	_obstacled_trajectory = *msg;
	updateGL();
	return;
}

void
GUI::
handleChosenTrajectoryLine( const tcl::TrajectoryLine::ConstPtr& msg ){
	_chosen_trajectory_line = *msg;
	updateGL();
	return;
}

void
GUI::
handleProjection( const nav_msgs::Path::ConstPtr& msg ){
	std::cout << "in handle projection" << std::endl;
	_projection = *msg;
	return;
}

void
GUI::
handleLookahead( const geometry_msgs::Point::ConstPtr& msg ){
	_lookahead = *msg;
	return;
}


void
GUI::
handleRandPoint( const geometry_msgs::Point::ConstPtr& msg ){
	_rand_point = *msg;
	return;
}


void
GUI::
handleNearPoint( const geometry_msgs::Point::ConstPtr& msg ){
	_near_point = *msg;
	return;
}

void
GUI::
handleNewPoint( const geometry_msgs::Point::ConstPtr& msg ){
	_new_point = *msg;
	return;
}


void
GUI::
handleRandomPoints( const geometry_msgs::Polygon::ConstPtr& msg ){
	_random_points = *msg;
	return;
}


void
GUI::
handleProjectedObstacles( const sim::RoundObstacles2D::ConstPtr& msg ){
	_projected_obstacles = *msg;
	updateGL();
	return;
}


void
GUI::
handleMap( const map_msgs::OccupancyGridUpdate::ConstPtr& msg ){
	_map = *msg;
	updateGL();
	return;
}

void
GUI::
handleGoalPt( const geometry_msgs::Point::ConstPtr& msg ){
	_goal_pt = *msg;
	return;
}


void
GUI::
handleClosePt( const geometry_msgs::Point::ConstPtr& msg ){
	_close_pt = *msg;
	return;
}

void 
GUI::
handleRRTStart( const path_planner::RRT::ConstPtr& msg ){
	_rrt_start = *msg;
	updateGL();
	return;
}

void
GUI::
timer_callback( void ){
	ros::spinOnce();
	return;
}

void
GUI::
initializeGL(){
	glClearColor( 1.0 , 1.0 , 1.0 , 1.0 );
	glEnable( GL_LINE_SMOOTH );
	glEnable( GL_BLEND );
}

void
GUI::
resizeGL( int width, int  height ){
	glViewport( 0 , 0 , width , height );
	return;
}

void
GUI::
paintGL(){
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	double ratio = (double) (size().width()) / (double) (size().height());
	gluOrtho2D( -_zoom * ratio + _center.first, _zoom * ratio + _center.first, -_zoom + _center.second , _zoom + _center.second );
	glMatrixMode(GL_MODELVIEW);

	if( !_odom_received ){
		_odom.pose.pose.position.x = 0.0;
		_odom.pose.pose.position.y = 0.0;
		_odom.pose.pose.position.z = 0.0;
		_odom.pose.pose.orientation = yaw_to_quaternion( 0.0 );
	}

	glLoadIdentity();
	drawMap( _map , 1.0 , 1.0 , 1.0 );
	drawGrid();
	drawCoordinateSystem();
	drawObstacles( _projected_obstacles , 1.0 , 1.0 , 0.0 , 1.0 );
	drawRRT( _rrt_start , 0.0 , 0.0 , 1.0 , 1.0 );
	drawLaserScan( _laserscan, 0.0 , 0.0 , 1.0);
	drawRobot( _odom.pose.pose, 0.0 , 0.0 , 0.0 , 0.1225 );
	drawRobot( _goal, 0.0 , 1.0 , 0.0 , 0.1225 );
	drawPath( _path , 1.0 , 0.0 , 0.0 , 1.0 );
	//drawPath( _projection, 1.0 , 0.0 , 1.0 , 1.0 );
	drawTrajectory( _trajectory , 1.0 , 0.0 , 1.0 , 1.0 );
	drawTrajectoryLine( _chosen_trajectory_line , 1.0 , 1.0 , 0.0, 1.0 );
	drawPolygon( _obstacle, 0.0 , 0.0 , 0.0 , 1.0 );
	drawPoints( _random_points , 1.0 , 0.0 , 0.0 , 5.0 );
	drawPoint( _goal_pt , 0.0 , 0.0 , 0.0 , 5.0 );
	// drawPoint( _rand_point , 0.0 , 0.0 , 0.0 , 5.0 );
	// drawPoint( _near_point , 1.0 , 0.0 , 0.0 , 5.0 );
	// drawPoint( _new_point , 1.0 , 1.0 , 0.0 , 5.0 );
	drawLine( _rand_point , _near_point , 1.0 , 0.0 , 0.0 , 1.0 );
	drawLine( _near_point , _new_point , 1.0 , 1.0 , 0.0 , 1.0 );
	drawPoint( _close_pt , 0.0 , 1.0 , 0.0 , 5.0 );
	drawPoint( _lookahead, 0.0 , 1.0 , 1.0 , 5.0 );
	return;
}

void
GUI::
drawCoordinateSystem(void){
	glBegin(GL_LINES);
	glColor4f( 1.0 , 0.0 , 0.0 , 1.0 );
	glVertex3f( 0.0 , 0.0 , 0.0 );
	glVertex3f( 1.0 , 0.0 , 0.0 );
	glColor4f( 0.0 , 1.0 , 0.0 , 1.0 );
	glVertex3f( 0.0 , 0.0 , 0.0 );
	glVertex3f( 0.0 , 1.0 , 0.0 );
	glColor4f( 0.0 , 0.0 , 1.0 , 1.0 );
	glVertex3f( 0.0 , 0.0 , 0.0 );
	glVertex3f( 0.0 , 0.0 , 1.0 );
	glEnd();
	return ;
}

void
GUI::
drawGrid(void){
	glColor4f( 0.8 , 0.8 , 0.8 , 1.0 );
	glLineWidth( 2.0 );
	glBegin( GL_LINES );
	for(int i = -10 ; i <= 10 ; i++){
		glVertex3f( -10.0 , (double)(i) , 0.0 );
		glVertex3f( 10 , (double)(i) , 0.0 );
		glVertex3f( (double)(i) , -10.0 , 0.0 );
		glVertex3f( (double)(i) , 10.0 , 0.0 );
	}
	glEnd();
	glLineWidth( 1.0 );
	return;
}

void
GUI::
drawLaserScan( const sensor_msgs::LaserScan& laser_scan , const double& red , const double& green , const double& blue ){
	glPushMatrix();
	glTranslated( _odom.pose.pose.position.x, _odom.pose.pose.position.y, 0.0 );
	glRotated( quaternion_to_yaw( _odom.pose.pose.orientation ) * 180.0 / M_PI, 0.0, 0.0, 1.0 );
	glColor4f( 1.0, 0.0, 0.0, 1.0 );
	glLineWidth( 2.0 );
	glBegin( GL_LINES );
	for( unsigned int i = 0; i < _laserscan.ranges.size(); i++ ){
		double angle = _laserscan.angle_min + ( double )( i ) * _laserscan.
		angle_increment;
		glVertex3f( 0.0, 0.0, 0.0 );
		glVertex3f( _laserscan.ranges[ i ] * cos( angle ), _laserscan.ranges[ i ] * sin( angle ), 0.0 );
	}
	glEnd(); 
	glLineWidth( 1.0 ); 
	glPopMatrix();
	
	return;
}

void
GUI::
drawRobot( const geometry_msgs::Pose& pose , const double& red , const double& green , const double& blue , const double& radius ){
	glPushMatrix();
	glTranslated( pose.position.x , pose.position.y , 0.0 );
	glRotated( quaternion_to_yaw( pose.orientation ) * 180.0 / M_PI, 0.0 , 0.0 , 1.0 );
	unsigned int descretization = 33;
	glColor4f( red , green , blue , 1.0 );
	glLineWidth( 5.0 );
	glBegin( GL_LINE_STRIP );
	for( unsigned int i = 0; i < descretization; i++ ){
		double angle = 2.0 * M_PI * (double) (i) / (double) (descretization - 1) ;
		glVertex3f( radius * cos(angle), radius * sin(angle) , 0.0 );
	}
	glEnd();
	glBegin( GL_LINES );
	glVertex3f( radius , 0.0 , 0.0 );
	glVertex3f( -radius, 0.0 , 0.0 );
	glEnd();
	glBegin( GL_TRIANGLES );
	glVertex3f( radius , 0.0 , 0.0 );
	glVertex3f( radius/4.0 , radius/2.0 , 0.0 );
	glVertex3f( radius/4.0 , -radius/2.0, 0.0 );
	glEnd();
	glLineWidth( 1.0 );
	glPopMatrix();
	return ;
}

void
GUI::
drawPath( const nav_msgs::Path& path , const double& red , const double& green, const double& blue, const double& width ){
	glLineWidth( width );
	glColor4f( red , green , blue , 1.0 );
	glBegin( GL_LINE_STRIP );
	for(unsigned int i = 0; i < path.poses.size(); i++){
		glVertex3f( path.poses[ i ].pose.position.x , path.poses[ i ].pose.position.y, 0.0 );
	}
	glEnd();
	glLineWidth(1.0);
	return ;
}


void
GUI::
drawTrajectoryLine( const tcl::TrajectoryLine& trajectory_line, const double& red, const double& green, const double& blue, const double& width){
	glLineWidth( width );
	glColor4f( red , green , blue , 1.0 );
	glBegin( GL_LINE_STRIP );
	for(unsigned int i = 0; i < trajectory_line.points.size(); i++){
		if( trajectory_line.points[i].cost >= 1 ){
			glColor4f( 0.0 , 1.0 , 1.0 , 1.0 );
		}
		else {		
			glColor4f( red , green , blue , 1.0 );
		}
		glVertex3f( trajectory_line.points[i].pose.x , trajectory_line.points[i].pose.y , 0.0 );
	}
	glEnd();	
	glLineWidth(1.0);
	return;
}


void 
GUI::
drawTrajectory( const tcl::Trajectory& trajectory , const double& red , const double& green , const double& blue, const double& width ){
	for( unsigned int i = 0; i < trajectory.lines.size() ; i++) {
		drawTrajectoryLine( trajectory.lines[i] , red , green , blue , width );
	}
	return;
}

void
GUI::
drawPolygon( const geometry_msgs::Polygon& polygon, const double& red , const double& green , const double& blue , const double& width ){
	if( polygon.points.size() <= 0 ) return;
	glLineWidth( width );
	glColor4f( red , green , blue , 1.0 );
	glBegin( GL_LINE_STRIP );
	for( unsigned int i = 0; i < polygon.points.size(); i++){
		glVertex3f( polygon.points[i].x , polygon.points[i].y , polygon.points[i].z );
	}
	glVertex3f( polygon.points[0].x , polygon.points[0].y , polygon.points[0].z );
	glEnd();
	glLineWidth( 1.0 );
	return;
}




void
GUI::
drawPoint( const geometry_msgs::Point& point, const double& red, const double& green , const double& blue , const double& size ){
	glPointSize( size );
	glBegin( GL_POINTS );
	glColor4f( red , green , blue , 1.0 );
	glVertex3f( point.x , point.y, point.z );
	glEnd();
	glPointSize( 1 );
	return;
}



void
GUI::
drawPoints( const geometry_msgs::Polygon& points , const double& red , const double& green , const double& blue, const double& size ){
	for( unsigned int i = 0; i < points.points.size(); i++ ){
		glPointSize( size );
		glColor4f( red , green , blue , 1.0 );
		glBegin( GL_POINTS );
		for( unsigned int i = 0; i < points.points.size(); i++){
			glVertex3f( points.points[i].x , points.points[i].y , points.points[i].z );
		}
		//glVertex3f( points.points[0].x , points.points[0].y , points.points[0].z );
		glEnd();
		glPointSize( 1 );
		return;
	}
}


void
GUI::
drawMap( const map_msgs::OccupancyGridUpdate& map, const double& r, const double& g, const double& b ){
	double discretization = 0.1;
	double half_discretization = discretization / 2.0;
	double min_x = -( double )( map.width - 1 ) * half_discretization; double min_y = -( double )( map.height - 1 ) * half_discretization;
	glPushMatrix();
	glBegin( GL_QUADS );
	for( unsigned int i = 0; i < map.width; i++ ){
		double x = min_x + ( double )( i ) * discretization; 
		for( unsigned int j = 0; j < map.height; j++ ){
			double y = min_y + ( double )( j ) * discretization;
			double occ = 1.0 - ( 1.0 / ( 1.0 + exp( ( double )( map.data[ i * map.height + j ] ) * 0.05 ) ) );
			glColor4f( ( 1.0 - occ ) * r, ( 1.0 - occ ) * g, ( 1.0 - occ ) * b, 1.0);
			glVertex3f( x - half_discretization,y - half_discretization,0.0 );
			glVertex3f( x + half_discretization,y - half_discretization,0.0 );
			glVertex3f( x + half_discretization,y + half_discretization,0.0 );
			glVertex3f( x - half_discretization,y + half_discretization,0.0 );
		}
	}
	glEnd(); 
	glPopMatrix(); 
	return;
}

void
GUI::
drawRRT( const path_planner::RRT& tree , const double& red, const double& green , const double& blue , const double& width ){
	glLineWidth( width );
	glColor4f( red , green , blue , 1.0 );
	glBegin( GL_LINES );
	for(unsigned int i = 0; i < tree.edges.size(); i++ ){
			glVertex3f( tree.edges[i].p1.x , tree.edges[i].p1.y , tree.edges[i].p1.z );
			glVertex3f( tree.edges[i].p2.x , tree.edges[i].p2.y , tree.edges[i].p2.z );

	}
	glEnd();
	glLineWidth(1.0);
	return;
}

void
GUI::
drawLine( const geometry_msgs::Point& p1 , const geometry_msgs::Point& p2 , const double& red , const double& green , const double& blue , const double& width ){
	glLineWidth( width );
	glColor4f( red , green , blue , 1.0 );
	glBegin( GL_LINES );
	glVertex3f( p1.x , p1.y , 0.0 );
	glVertex3f( p2.x , p2.y , 0.0 );
	glEnd();
	glLineWidth(1.0);
	return;
}

void
GUI::
drawObstacle( const sim::RoundObstacle2D& ob , const double& red , const double& green , const double& blue , const double& width ){
	glPushMatrix();
	glTranslated( ob.pose.x , ob.pose.y , 0.0 );
	glColor4f( red , green , blue , 1.0 );
	unsigned int descretization = 33;
	glLineWidth( 5.0 );
	glBegin( GL_LINE_STRIP );
	for( unsigned int i = 0; i < descretization; i++ ){
		double angle = 2.0 * M_PI * (double) (i) / (double) (descretization - 1);
		glVertex3f( ob.radius * cos(angle) , ob.radius * sin(angle) , 0.0 );
	}
	glEnd();
	glLineWidth( 1.0 );
	glPopMatrix();
}

void
GUI::
drawObstacles( const sim::RoundObstacles2D& obs , const double& red , const double& green , const double& blue , const double& width ){
	for(unsigned int i = 0; i < obs.obstacles.size(); i++){
		drawObstacle( obs.obstacles[i] , red , green , blue , width );
	}
}

void
GUI::
handle_run_button(){
	run_button->setText("running");
}


void
GUI::
keyPressEvent( QKeyEvent * event ){
	if( event->matches( QKeySequence::Copy ) ){
		close();
		return ;
	}
	else{
		switch( event->key() ){
			case Qt::Key_Left:
				_center.first -= 0.5;
				break;
			case Qt::Key_Right:
				_center.first += 0.5;
				break;
			case Qt::Key_Up:
				_center.second += 0.5;
				break;
			case Qt::Key_Down:
				_center.second -= 0.5;
				break;
			case Qt::Key_I:
				if( _zoom > 0.5 ){
					_zoom -= 0.5;
				}
				break;
			case Qt::Key_O:
				_zoom += 0.5;
				break;
			default:
				std::cout << "could not handle key " << event->key() << std::endl;
				break;
		}
		updateGL();
	}
	return;
}



