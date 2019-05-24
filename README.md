Overview:
	
	-This project is a simulation to represent different approaches taken in obstacle avoidance while driving. The two
	algorithms are input space sampling and RRT. Input space sampling is introduced in the directory /src/tcl works by
	generating a forward model for the car to define a set of trajectories. These trajectories are done by sampling
	commands in a future states periodically for a specified period of time. The RRT approach is short for Rapidly
	exploring Random Trees. That approach is developed to explore infinite grid where there is not a defined space and it
	explores in a random way. The inplementation done in the directory /src/path_planner is of the RRT* algorithm. That
	algorithm is more optimal than the regular RRT where it optimizes on the local paths for the surrounding areas around
	the explored point. 
