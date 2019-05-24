
#include "sim/obstacles.h"
#include "sim/RoundObstacle2D.h"
#include <time.h>


sim::RoundObstacle2D
make_round_obstacle( const double& x , const double& y , const double& theta, const double& radius , const double& linear , const double& angular ){
	sim::RoundObstacle2D round_obstacle2d;
	round_obstacle2d.pose.x = x;
	round_obstacle2d.pose.y = y;
	round_obstacle2d.pose.theta = theta;
	round_obstacle2d.radius = radius;
	round_obstacle2d.linear = linear;
	round_obstacle2d.angular = angular;
	return round_obstacle2d;
}


Obstacles::
Obstacles()
	:number_of_obstacles( 6 ),
	 max_speed( 0.5 ),
	 max_curvature( 0.5 ){
		srand(time(NULL));
		for(unsigned int i = 0 ; i < number_of_obstacles ; i++ ){
			// round_obstacles2d.obstacles.push_back( sim::RoundObstacle2D() );
			double x = 0;
			double y = 0;
			// if( rand() % 2 ){
			if(1){
				y = ((((double) (rand() % 400)) / 100.0 ) + 1.0 ); 
				x = ((((double) (rand() % 400)) / 100.0 ) + 1.0 ); 
			}
			else{
				y = -1.0 * ((((double) (rand() % 400)) / 100.0 ) + 1.0 ); 
				x = -1.0 * ((((double) (rand() % 400)) / 100.0 ) + 1.0 ); 
			}
			double theta = ( ((double) (rand() % 200)) / 100.0 ) * M_PI;
			double radius = ( ((double) (rand() % 100)) / 100.0 ) + 0.25;
			double linear = ( ((double) (rand() % 100 )) / 400.0 ) + 0.1;
			double angular = ( ((double) (rand() % 100 )) / 400.0 ) + 0.1;
			std::cout << "(x,y):(" << x << "," << y << ")" << std::endl;

			round_obstacles2d.obstacles.push_back( make_round_obstacle( x , y , theta , radius , linear , angular ) );
		}

	}


Obstacles::
~Obstacles(){}


void
Obstacles::
step( const double& dt ){
	for( unsigned int i = 0; i < round_obstacles2d.obstacles.size(); i++ ){
		round_obstacles2d.obstacles[i].pose.x = round_obstacles2d.obstacles[i].pose.x + ( round_obstacles2d.obstacles[i].linear * dt * cos( round_obstacles2d.obstacles[i].pose.theta ) );
		round_obstacles2d.obstacles[i].pose.y = round_obstacles2d.obstacles[i].pose.y + ( round_obstacles2d.obstacles[i].linear * dt * sin( round_obstacles2d.obstacles[i].pose.theta ) );
		round_obstacles2d.obstacles[i].pose.theta = round_obstacles2d.obstacles[i].pose.theta + ( round_obstacles2d.obstacles[i].angular * dt );
	}
}

void
Obstacles::
update(){
	for(unsigned int i = 0; i < round_obstacles2d.obstacles.size(); i++ ){
		if(rand() % 2){
			round_obstacles2d.obstacles[i].linear = round_obstacles2d.obstacles[i].linear + (((double) ( rand() % 200 ) ) / 1000.0 );
		}
		else{
			round_obstacles2d.obstacles[i].linear = round_obstacles2d.obstacles[i].linear - (((double) ( rand() % 200 ) ) / 1000.0 );
		}
		if(rand() % 2){
			round_obstacles2d.obstacles[i].linear = round_obstacles2d.obstacles[i].angular + (((double) ( rand() % 200 ) ) / 1000.0 );
		}
		else{
			round_obstacles2d.obstacles[i].linear = round_obstacles2d.obstacles[i].angular - (((double) ( rand() % 200 ) ) / 1000.0 );
		}

		if( round_obstacles2d.obstacles[i].linear >= max_speed ){
			round_obstacles2d.obstacles[i].linear = max_speed;
		}
		if( round_obstacles2d.obstacles[i].angular >= max_curvature ){
			round_obstacles2d.obstacles[i].angular = max_curvature;
		}
		
	}
}

