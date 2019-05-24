#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Polygon.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "path_planner/RRT.h"
#include "geometry_msgs/Point.h"
#include "sim/RoundObstacles2D.h"

#include "ros/ros.h"



enum
TreeState{
	Reached,
	Advanced,
	Trapped
};

typedef enum TreeState TreeState;


class
RRTPose{
	public:
		RRTPose( const geometry_msgs::Point& _point , const geometry_msgs::Point& _goal , RRTPose* _prev);
		virtual ~RRTPose();

		int get_h(const geometry_msgs::Point& end );
		void update_g( RRTPose* new_prev );

		geometry_msgs::Point point;
		int g;
		int f;
		int h;
		RRTPose* prev;
};
	
class Comp{
	public:
	bool operator()(const RRTPose* p1 , const RRTPose* p2 ) const {
		return p1->f > p2->f;
	}
};	

class
RRTNode{
	public:
		RRTNode();
		RRTNode( const geometry_msgs::Point& _point );
		RRTNode( const geometry_msgs::Point& _point , const double& cost);
		RRTNode( const geometry_msgs::Point& _point , const double& cost , RRTNode* parent);
		virtual ~RRTNode();

		geometry_msgs::Point point;
		double cost;
		RRTNode* parent;
		std::vector< RRTNode > children;
		bool is_goal;
};

class
RRTPathPlanner{
	public:
		RRTPathPlanner();
		virtual ~RRTPathPlanner();


		void handle_odom( const nav_msgs::Odometry::ConstPtr& msg );
		void handle_goal( const geometry_msgs::Pose::ConstPtr& msg );
		void handle_map( const map_msgs::OccupancyGridUpdate::ConstPtr& msg );
		void handle_obstacles( const sim::RoundObstacles2D::ConstPtr& msg );

		bool is_point_occupied( const geometry_msgs::Point& p );
		bool create_new_state( RRTNode& node , const geometry_msgs::Point& x, const geometry_msgs::Point& x_near, geometry_msgs::Point& x_new ,const geometry_msgs::Point& goal_pt , double& edge_distance , const double& bias);
		TreeState extend_tree( RRTNode& node, const geometry_msgs::Point& x , const geometry_msgs::Point& goal_pt , const double& bias);
		RRTNode build_rrt( const geometry_msgs::Point& x_init , const geometry_msgs::Point& goal_pt , const unsigned int& K , const double& bias);
		void search( const geometry_msgs::Pose& start , const geometry_msgs::Pose& goal );
		nav_msgs::Path construct_path( const RRTNode& tree );
		void update_obstacles_location( const double time );
		void update_tree();
		// void search_tree( const geometry_msgs::Point& start , const geometry_msgs::Point& goal_pt , std::vector<RRTPose*>& closedlist);
		// nav_msgs::Path construct_path(const geometry_msgs::Point& start, const geometry_msgs::Point& goal );

		int number_of_samples;
		bool odom_recieved;
		bool map_recieved;
		bool obstacle_recieved;
		double discretization;
		double free_cell;
		double delta;
		double bias;
		double neighboring;
		ros::Publisher rrt_publisher;
		ros::Publisher path_publisher;
		ros::Publisher new_rand_publisher;
		ros::Publisher new_point_publisher;
		ros::Publisher near_point_publisher;
		ros::Publisher projected_obstacle_publisher;
		nav_msgs::Odometry odom;
		geometry_msgs::Pose goal;
		map_msgs::OccupancyGridUpdate map;
		path_planner::RRT _tree;
		nav_msgs::Path path;
		RRTNode root;
		sim::RoundObstacles2D obstacles;
		sim::RoundObstacles2D projected_obstacles;

};
