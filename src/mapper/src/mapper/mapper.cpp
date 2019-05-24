
#include "mapper/mapper.h"

double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 * ( quaternion.w * quaternion.z + quaternion.x * quaternion.y ), 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ) );
}

double
compute_distance( const double& x1 , const double& y1 , const double& x2 , const double& y2 ){
	return sqrt( ( ( x2 - x1 ) * ( x2 - x1 ) ) + ( ( y2 - y1 ) * ( y2 - y1 ) ) );
}

bool
row_col_in_map( const map_msgs::OccupancyGridUpdate& map, const int& row,
const int& col ){
	if( ( row < 0 ) || ( col < 0 ) ){
		return false;
	} else if ( ( row >= map.width ) || ( col >= map.height ) ){
		return false;
	} else {
		return true;
	}
}

int8_t
double_to_int8( const double& arg ){
	int tmp = ( int )( round( arg * 20.0 ) ); 
	if( tmp < -127 ){
		tmp = -127;
	} else if ( tmp > 127 ){
		tmp = 127; 
	}
	return ( int8_t )( tmp ); 
}

double
row_to_x( const map_msgs::OccupancyGridUpdate& map, const int& row, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.width ) - 1 ) / 2 +
	row ) );
}
double
col_to_y( const map_msgs::OccupancyGridUpdate& map, const int& col, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.height ) - 1 ) / 2 +
	col ) );
}
int
x_to_row( const map_msgs::OccupancyGridUpdate& map, const double& x, const double& discretization ){
	return round( x / discretization + ( double )( ( map.width - 1 ) / 2 ) );
}

int 
y_to_col( const map_msgs::OccupancyGridUpdate& map, const double& y, const double& discretization ){
	return round( y / discretization + ( double )( ( map.height - 1 ) / 2 ) );
}




Mapper::
Mapper( const double& discretization, const unsigned int& width, const unsigned int& height ) : 
	_discretization( discretization ),
	_xs( width ),
	_ys( height ),
	_l0( log( 0.5 / ( 1.0 - 0.5 ) ) ),
	_locc( log( 0.9 / ( 1.0 - 0.9 ) ) ),
	_lfree( log( 0.1 / ( 1.0 - 0.1 ) ) ),
	_odometry(),
	_scans(),
	_map() {
		_map.width = width;
		for( unsigned int i = 0; i < _map.width; i++ ){
			_xs[ i ] = row_to_x( _map, i, _discretization );
		}
		_map.height = height;
		for( unsigned int i = 0; i < _map.height; i++ ){
			_ys[ i ] = col_to_y( _map, i, _discretization );
		}
		_map.data.resize( _map.width * _map.height );
		for( unsigned int i = 0; i < _map.width * _map.height; i++ ){ 
			_map.data[ i ] = double_to_int8( _l0 );
		} 
	}

Mapper::
~Mapper(){
}

void
Mapper::
handleOdometry( const nav_msgs::Odometry::ConstPtr& msg ){
	_odometry = *msg;
	return;
}

void
Mapper::
handleLaserScan( const sensor_msgs::LaserScan::ConstPtr& msg ){
	_scans.push_back( *msg );
	return;
}

void
Mapper::
update( void ){
	if( !_scans.empty() ){
		double x = _odometry.pose.pose.position.x;
		double y = _odometry.pose.pose.position.y;
		double yaw = quaternion_to_yaw( _odometry.pose.pose.orientation );
		std::cout << "updating " << _scans.size() << " scans" << std::endl;
		for( unsigned int i = 0; i < _scans.size(); i++ ){
			std::vector< int > occupied_cells;
			std::vector< int > free_cells;
			// search over all ranges
			for( unsigned int j = 0; j < _scans[ i ].ranges.size(); j++ ){
				double scan_angle = _scans[ i ].angle_min + j * _scans[ i ].angle_increment;
				// check for occupied cells
				if( fabs( _scans[ i ].ranges[ j ] - _scans[ i ].range_max ) > 0.1 ){
					double scan_x = x + _scans[ i ].ranges[ j ] * cos( yaw + scan_angle);
					double scan_y = y + _scans[ i ].ranges[ j ] * sin( yaw + scan_angle);
					int row = x_to_row( _map, scan_x, _discretization );
					int col = y_to_col( _map, scan_y, _discretization );
					if( row_col_in_map( _map, row, col ) ){
						int index = row * _map.height + col;
						if( find( occupied_cells.begin(), occupied_cells.end(), index ) == occupied_cells.end() ){
							occupied_cells.push_back( index );
						}
					}
				}
				double scan_distance = 0.0;
				double scan_increment = 0.1;
				// check for free cells
				while( scan_distance < _scans[ i ].ranges[ j ] ){
					double scan_x = x + scan_distance * cos( yaw + scan_angle );
					double scan_y = y + scan_distance * sin( yaw + scan_angle );
					int row = x_to_row( _map, scan_x, _discretization );
					int col = y_to_col( _map, scan_y, _discretization );
					if( row_col_in_map( _map, row, col ) ){
						int index = row * _map.height + col;
						if( ( find( free_cells.begin(), free_cells.end(), index ) ==
							free_cells.end() ) && ( find( occupied_cells.begin(),
							occupied_cells.end(), index ) == occupied_cells.end() ) ){
							free_cells.push_back( index );
						}
					}
					scan_distance += scan_increment;
				}
			}
			// updating free _map log probability
			for( unsigned int j = 0; j < free_cells.size(); j++ ){
				// implement this
			}
			// updating occupied _map log probability
			for( unsigned int j = 0; j < occupied_cells.size(); j++ ){
				// implement this
			}
		}
	}
	_scans.clear();
	return;
}

bool
Mapper::
checkMap( const double& x,const double& y, const double& radius, const double& threshold ){
	int row = x_to_row( _map, x, _discretization );
	int col = y_to_col( _map, y, _discretization );
	int offset = ceil( radius / _discretization );
	for( int i = -offset; i < offset; i++ ){
		double dx = row_to_x( _map, row + i, _discretization ) - x;
		for( int j = -offset; j < offset; j++ ){
			double dy = col_to_y( _map, col + j, _discretization ) - y;
			if( row_col_in_map( _map, row + i, col + j ) && ( sqrt( dx * dx + dy *dy ) < radius ) ){
				int index = ( row + i ) * _map.height + col + j;
				if( _map.data[ index ] > threshold ){
					return false;
				}
			}
		}
	}
	return true;
}

/*
void
Mapper::
handle_obstacles( const geometry_msgs::Polygon::ConstPtr& msg ){
	_obstacles = *msg;
	for( unsigned int col = 0; col < _map.width; col++ ){
		for( unsigned int row = 0; row < _map.height; row++ ){
			double x = row_to_x( _map , row , _discretization );
			double y = col_to_y( _map , col , _discretization );
			for( unsigned int i = 0; i < _obstacles.points.size(); i++ ){
				double distance = compute_distance( x , y , _obstacles.points[i].x , _obstacles.points[i].y );
				if( distance < _obstacles.points[i].z ){
					_map.data[ ( col * _map.width ) + row ] = _locc;
				}
			}
		}
	}
	// fill_line();
	map_publisher.publish( _map );
	return;
}
*/

void
Mapper::
shadow_map(){
	for( unsigned int col = 0; col < _map.width; col++ ){
		for( unsigned int row = 0; row < _map.height; row++ ){
			double x = row_to_x( _map , row , _discretization );
			double y = col_to_y( _map , col , _discretization );
			bool filled = false;
			for( unsigned int i = 0; i < _obstacles.obstacles.size(); i++ ){
				double distance = compute_distance( x , y , _obstacles.obstacles[i].pose.x , _obstacles.obstacles[i].pose.y );
				if( distance < _obstacles.obstacles[i].radius ){
					filled = true;
					break;
				}
			}
			if( filled ){
				_map.data[ ( col * _map.width ) + row ] = double_to_int8( _locc );
			}
			else{
				_map.data[ ( col * _map.width ) + row ] = double_to_int8( _lfree );
			}
		}
	}
}

void
Mapper::
handle_obstacles( const sim::RoundObstacles2D::ConstPtr& msg ){
	_obstacles = *msg;
	std::cout << "obstacles recieved of size : " << _obstacles.obstacles.size() << std::endl;
	//_map.data.clear();
	shadow_map();
	// fill_line();
	map_publisher.publish( _map );
	return;
}


void
Mapper::
add_obstacle( const double& x1 , const double& x2 , const double& y1 , const double& y2 ){
	int x_b = x_to_row( _map , x1 , _discretization );
	int x_e = x_to_row( _map , x2 , _discretization );
	int y_b = y_to_col( _map , y1 , _discretization );
	int y_e = y_to_col( _map , y2 , _discretization );
	for( int i = x_b; i< x_e ; i++){
		for(int j = y_b; j < y_e; j++){
			_map.data[ i * _map.width + j ] = double_to_int8( _locc );
		}
	}
	return;
}

/*
void
Mapper::
fill_line(){
	if( _obstacles.points.empty() ) return;
	std::cout << "filling line" << std::endl;
	// double x1 = 2.0;// _obstacles.points[i].x;
	// double y1 = 2.0;//_obstacles.points[i].y;
	// double x2 = 1.0;//_obstacles.points[i-1].x;
	// double y2 = 1.0;//_obstacles.points[i-1].y;
	for(unsigned int row = 0; row < _map.height; row++){
		for(unsigned int col = 0; col < _map.width; col++){
			double x = row_to_x( _map , row , _discretization );
			double y = col_to_y( _map , col , _discretization );
			int intersections = 0;
			// std::cout << "checking for all " << _obstacles.points.size() << " points " << std::endl;
			for(unsigned int i = 0 ; i < _obstacles.points.size() ; i++ ){

			 double x1 = _obstacles.points[i].x;
			 double y1 = _obstacles.points[i].y;
			 double x2 = 0.0;
			 double y2 = 0.0;
			 if( i == _obstacles.points.size() -1 ){
				 x2 = _obstacles.points[0].x;
				 y2 = _obstacles.points[0].y;
			 }
			 else{
				 x2 = _obstacles.points[i+1].x;
				 y2 = _obstacles.points[i+1].y;
			 }
			 double m = ( y2 - y1 ) / ( x2 - x1 );
			 if( x <= std::max( x1 , x2 ) && x >= std::min( x1 , x2 ) && y <= std::max( y1 , y2 ) && y >= std::min( y1 , y2 ) ){
				 _map.data[ row * _map.width + col ] = double_to_int8( _locc );
				 break;
			 }
			 else if( ( ( ( x < std::max( x1 , x2 ) ) && ( x > std::min( x1 , x2 ) ) ) || ( ( y < std::max( y1 , y2 ) ) && ( y > std::min( y1 , y2 ) ) ) ) && ( m == 0 ) && ( ( ( y - y1 ) - ( m * ( x - x1 ) ) ) > 0 ) ){
			 // else if( ( ( ( y - y1 ) - ( m * ( x - x1 ) ) ) < 0 ) ){
					// std::cout << "found an intersection\n";
					// std::cout << "found places to fill at(col,row):(" << col << "," << row <<") and (x,y):(" << x << "," << y << ")\n" ;
				 intersections++;
				 //_map.data[ row * _map.width + col ] = double_to_int8( _locc );
				}
			}
			// _map.data[ row * _map.width + col ] = double_to_int8( _locc );
			// std::cout << "(col,row):(" << col  << "," << row << "):::> " << _map.data[ row * _map.width + col ] << std::endl;
			if( intersections % 2 == 1 ){
					// std::cout << "found places to fill at(col,row):(" << col << "," << row <<")\n" ;
				_map.data[ row * _map.width + col ] = double_to_int8( _locc );
			}
		}
	}
	std::cout << "done creating\n";
	return;
}

*/
std::ostream&
operator<<( std::ostream& out,const Mapper& other ){
	return out;
}
