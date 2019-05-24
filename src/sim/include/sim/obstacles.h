
#include "sim/RoundObstacles2D.h"

class Obstacles{
	public:
		Obstacles();
		virtual ~ Obstacles();

		void step(const double& dt);
		void update();


		int number_of_obstacles;
		double max_speed;
		double max_curvature;
		sim::RoundObstacles2D round_obstacles2d;

		
};
