#include "path_planner/rrt_path_planner.h"
#include "geometry_msgs/Pose2D.h"
#include <ctime>
#include <stdio.h>
#include <string>

#include <Eigen/Dense>

void make_rrt_from_tree( path_planner::RRT& , RRTNode& );

double
compute_distance( const geometry_msgs::Pose2D& p1 , const geometry_msgs::Pose2D& p2 )
{
	return sqrt(((p1.x - p2.x)*(p1.x - p2.x))+((p1.y - p2.y)*(p1.y - p2.y)));
}

double
compute_distance( const geometry_msgs::Point& p1 , const geometry_msgs::Point& p2 )
{
	return sqrt(((p1.x - p2.x)*(p1.x - p2.x))+((p1.y - p2.y)*(p1.y - p2.y)));
}

double
compute_angle( const geometry_msgs::Point& p1 , const geometry_msgs::Point& p2 , const geometry_msgs::Point& p3 ){
	Eigen::Vector3d v1( p1.x-p2.x , p1.y-p2.y , p1.z-p2.z );
	Eigen::Vector3d v2( p3.x-p2.x , p3.y-p2.y , p3.z-p2.z );
	return acos( v1.dot(v2) / (v1.norm() * v2.norm()) );
}


bool
operator== ( const geometry_msgs::Point& p1, const geometry_msgs::Point& p2 ){
	return (p1.x == p2.x) && (p1.y == p2.y);
}


std::ostream&
operator<< ( std::ostream& out , const geometry_msgs::Point& point ){
	out << " (x,y): (" << point.x << "," << point.y << ")";
}


double
quaternion_to_yaw( const geometry_msgs::Quaternion& quaternion ){
	return atan2( 2.0 *  (quaternion.w * quaternion.z + quaternion.x * quaternion.y) , 1.0 - 2.0 * ( quaternion.y * quaternion.y + quaternion.z * quaternion.z ));
}

geometry_msgs::Quaternion
yaw_to_quaternion( const double& yaw ){
	geometry_msgs::Quaternion quaternion;
	quaternion.w = cos( yaw / 2.0 );
	quaternion.x = 0.0;
	quaternion.y = 0.0;
	quaternion.z = sin( yaw / 2.0 );
	return quaternion;
}

geometry_msgs::Pose2D
pose_to_pose2d( const geometry_msgs::Pose& pose )
{
	geometry_msgs::Pose2D pose2d;
	pose2d.x = pose.position.x;
	pose2d.y = pose.position.y;
	pose2d.theta = quaternion_to_yaw( pose.orientation );
	return pose2d;
}  

geometry_msgs::PoseStamped
pose_to_pose_stamped( const geometry_msgs::Pose& pose ){
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.pose = pose;
	// std::cout << "here" << std::endl;
	return pose_stamped;
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
	return ( discretization * ( double )( -( ( int )( map.width ) - 1 ) / 2 + row ) );
}


double
col_to_y( const map_msgs::OccupancyGridUpdate& map, const int& col, const double& discretization ){
	return ( discretization * ( double )( -( ( int )( map.height ) - 1 ) / 2 + col ) );
}


int
y_to_col( const map_msgs::OccupancyGridUpdate& map, const double& y, const double& discretization ){
	return round( y / discretization + ( double )( ( map.height - 1 ) / 2 ) );}

int
x_to_row( const map_msgs::OccupancyGridUpdate& map, const double& x, const double& discretization ){
	return round( x / discretization + ( double )( ( map.width - 1 ) / 2 ) );
}


RRTPose::
RRTPose(const geometry_msgs::Point& _point , const geometry_msgs::Point& _goal , RRTPose* _prev)
	:point( _point ),
	 prev( _prev )
	 {
		 if(!prev) g = 0.0;
		 else g = prev->g + compute_distance( prev->point , point );
		 h = get_h( _goal );
		 f = g+h;

	 }

RRTPose::
~RRTPose(){}


int
RRTPose::
get_h( const geometry_msgs::Point& goal ){
	return compute_distance( point , goal );
}

void
RRTPose::
update_g( RRTPose* new_prev ){
	if(!new_prev || !prev) return;
	double new_g = compute_distance( new_prev->point , point ) + new_prev->g;
	if( new_g < g ){
		g = new_g;
		prev = new_prev;
		f = new_g + h;
	}
	return;
}

RRTNode::
RRTNode()
	: point( geometry_msgs::Point() ),
	  cost( 0.0 ),
		parent( NULL ),
		is_goal( false )
{
}

RRTNode::
RRTNode( const geometry_msgs::Point& _point )
	: point( _point ),
	  cost( 0.0 ),
		parent( NULL ),
		is_goal( false )
{

}

RRTNode::
RRTNode( const geometry_msgs::Point& _point , const double& cost )
	: point ( _point ),
	  cost( cost ),
		parent( NULL ),
		is_goal( false )
		{}

RRTNode::
RRTNode( const geometry_msgs::Point& _point , const double& cost , RRTNode* _parent )
	: point( _point ),
	  cost( cost ),
		parent( _parent )
		{}

RRTNode::
~RRTNode(){}

// draw a point completely at random
geometry_msgs::Point
randomly_picked(){
	geometry_msgs::Point p;
	if( rand() % 2 ){ // random positive x
		p.x = (double )(rand() % 10);
	}
	else{ // random negative x
		p.x =(-1.0)*(double)(rand() % 10);
	}
	if( rand() % 2 ){ // random positive y
		p.y = (double)(rand() % 10 );
	}
	else{ // random negative y
		p.y = (-1.0)*(double)(rand() % 10 );
	}
	return p;
}
		

//generate a random number with a bias percent of the time pick goal
geometry_msgs::Point
generate_random_point(const geometry_msgs::Point& goal_pt , const double& bias){
	geometry_msgs::Point p;
	//if I am biased 100 % of the time towards goal then I return goal
	if( bias == 1.0 ) return goal_pt;
	//if bias is 0% so I could pick goal
	if( bias == 0.0 ) return randomly_picked();
	// draw in a number at random to find the bias
	double random_draw = ((double)( rand() % 1000 )) / 1000.0;
	if( random_draw <= bias ){ // my draw is within the bias range
		return goal_pt;
	}
	return randomly_picked();
}


// find the nearest node in the tree.
RRTNode*
nearest_neighbor( RRTNode& node , const geometry_msgs::Point x ){
	std::cout << "Nearest Point " << std::endl;
	RRTNode* nearest = &node;
	if( node.children.size() == 0 ) return nearest;
	double min_distance = compute_distance( nearest->point , x );
	for(unsigned int i = 0; i < node.children.size(); i++){
		RRTNode* temp = nearest_neighbor( node.children[i] , x );
		std::cout << "temp is : " << temp->point << " nearest is : " << nearest->point << " x : " << x << std::endl;
		double distance = compute_distance( temp->point , x );
		std::cout << "distance : " << distance  << " , min_distance : " << min_distance << std::endl;
		if( distance < min_distance ){
			std::cout << "UPDATE VALUE" << std::endl;
			min_distance = distance;
			nearest = temp;
		}
	}
	std::cout << nearest->point << std::endl;
	return nearest;
}

bool
RRTPathPlanner::
is_point_occupied( const geometry_msgs::Point& p ){
	if(map.data.empty()) return false;
	int row = x_to_row( map , p.x , discretization );
	int col = y_to_col( map , p.y , discretization );
	if( map.data[ ( row * map.width ) + col ] > double_to_int8( free_cell ) ) return true;
	else return false;
}

bool
RRTPathPlanner::
create_new_state( RRTNode& node, const geometry_msgs::Point& x , const geometry_msgs::Point& x_near , geometry_msgs::Point& x_new , const geometry_msgs::Point& goal_pt , double& edge_distance , const double& bias){
	// std::cout << "create new state" << std::endl;
	double distance = compute_distance( x , x_near );
	if( distance <= delta ) {
		// std::cout << "distance: " << distance << std::endl;
		x_new.x = x.x;
		x_new.y = x.y;
		edge_distance = distance;
		return false;
	}
	Eigen::Vector2d v( ( x.x - x_near.x ) , ( x.y - x_near.y  ) );
	v = v * ( delta / distance );	
	// std::cout << "(0,1): (" << v(0) << "," << v(1) << ")" << std::endl;
	x_new.x = x_near.x + v(0);
	x_new.y = x_near.y + v(1);
	edge_distance = delta;
	if( is_point_occupied( x_new ) ){
		// std::cout << "point occupied where x_new: " << x_new << std::endl;
		geometry_msgs::Point x_temp = generate_random_point(goal_pt , bias);
		RRTNode* temp_nearest_node = nearest_neighbor( node , x_temp );
		return create_new_state( node , x_temp , x_near , x_new , goal_pt , edge_distance , bias);
	}
	return true;
}

/*
 * check if the number is close to goal or not
 */
bool
close_to_goal( const geometry_msgs::Point& x , const geometry_msgs::Point& goal_pt , const double& delta , double& edge_distance ){
	edge_distance = compute_distance( x , goal_pt );
	if( edge_distance <= delta ){
		return true;
	}
	return false;
}


/*
 * printing the edges in the tree for testing 
 */
void
print_tree( const path_planner::RRT& tree ){
	std::cout << "***************************************" << std::endl;
	std::cout << "***************************************" << std::endl;
	for(unsigned int i = 0; i < tree.edges.size(); i++ ){
		std::cout << "( " << tree.edges[i].p1 << " --- " << tree.edges[i].p2 << " )" <<  std::endl;
	}
	std::cout << "***************************************" << std::endl;
	std::cout << "***************************************" << std::endl;
}

/*
 * find the neighbors in a tree
 */
void
find_neighbors( RRTNode& node , const geometry_msgs::Point& x , std::vector< RRTNode* >& neighbors , const double& neighboring ){
	// check the leaves
	if( node.children.size() <= 0 ){
		if( compute_distance(x , node.point) <= neighboring ){
			// std::cout << "found neighbor at leaf " << std::endl;
			neighbors.push_back( &node );
		}
		return;
	}

	//check the current node
	if( compute_distance( x , node.point) <= neighboring ){
		// std::cout << "found neighbor at node : " << node.point << std::endl;
		neighbors.push_back( &node );
	}
	// apply the same for all children 
	for( unsigned int i = 0; i < node.children.size(); i++ ){
		find_neighbors( node.children[i] , x , neighbors , neighboring );
	}
	return;
}

/* 
 * given a parent and a one of it's children erase the child
 * from the list of children
 */
bool
erase_child( RRTNode* parent , RRTNode& node ){
	if( !parent ) return false;
	for(std::vector<RRTNode>::iterator it = parent->children.begin(); it != parent->children.end(); ++it){
		if( it->point == node.point ){
			parent->children.erase( it );
			return true;
		}
	}
	return false;
}


/*
 * add a node to the best outcome from a bunch of neighboring 
 * nodes
 */
RRTNode*
add_to_tree( RRTNode& node , const geometry_msgs::Point& x , const double& neighboring ){
	// find the neighbors in the tree
	std::vector< RRTNode* > neighbors;
	find_neighbors( node , x , neighbors , neighboring );

	// std::cout << "neighbors size : " << neighbors.size() << std::endl;
	
	// if only the nearest is threre then you got to just connect it
	if(neighbors.size() == 1){
		RRTNode* n = new RRTNode( x , neighbors[0]->cost + compute_distance( neighbors[0]->point , x ) ); 
		neighbors[0]->children.push_back(*n);
		return n;
	}
	double min_dist = HUGE_VAL;
	RRTNode* closest;
	unsigned int index = -1;

	// adding it to one of the neighbors
	for(unsigned int i = 0; i < neighbors.size(); i++ ){
		double distance = neighbors[i]->cost + compute_distance( x , neighbors[i]->point );
		if( distance < min_dist ){
			min_dist = distance;
			closest = neighbors[i];
			index = i;
		}
	}
	
	// add it to the tree
	RRTNode* n_new = new RRTNode( x , min_dist );
	closest->children.push_back( *n_new );
	// RRTNode* n = &closest->children.back();
	// neighbors.erase( neighbors.begin() + index );
	

	// check for optimality
	// min_dist = HUGE_VAL;
	for( unsigned int i = 0; i < neighbors.size(); i++ ){
	// std::cout << "i : " << i << " size : " <<  neighbors.size() <<  std::endl;
		double x_cost = n_new->cost + compute_distance( neighbors[i]->point , x );
		if( x_cost < neighbors[i]->cost ){

			if( erase_child( neighbors[i]->parent , *neighbors[i] ) ){
				neighbors[i]->cost = x_cost;
				neighbors[i]->parent = n_new;
				n_new->children.push_back( *neighbors[i] );
			}
		}

	}

	return n_new;
}

void
print_tree( const RRTNode& node, std::string str ){
	std::cout << node.point << std::endl;
	for(unsigned int i = 0; i < node.children.size(); i++){
		print_tree( node.children[i] , str + " " );
	}
}


/*
 * extending the tree with the node that is on the way of the
 * the random point to the nearest node on the tree
 */
TreeState
RRTPathPlanner::
extend_tree( RRTNode& node , const geometry_msgs::Point& x , const geometry_msgs::Point& goal_pt , const double& bias){
	// find the nearest node
	RRTNode* n_near = nearest_neighbor( node , x );
	
	geometry_msgs::Point x_new;
	double edge_distance = 0.0;
	// make a new node that is on the way of the node 
	if( create_new_state( node , x , n_near->point , x_new , goal_pt , edge_distance , bias)){
		// new_rand_publisher.publish(x);
		// near_point_publisher.publish(n_near->point);
		// new_point_publisher.publish(x_new);
		// std::cout << "x: " << x << " near: " << n_near->point << " x_new: " << x_new << std::endl;
		// std::cin.get();



		// attach the node to the tree
		// RRTNode new_node( x_new );
		// std::cout << "before adding to the tree " << std::endl;
		// std::cout << "********* PRINT TREE BEFORE ************" << std::endl;
		// print_tree( node , "" );
		// std::cout << "***************************************" << std::endl;
		RRTNode* n_new = add_to_tree( node , x_new , neighboring );
		// path_planner::RRT temp_rrt;
		// make_rrt_from_tree( temp_rrt , node );
		// std::cout << "********* PRINT TREE AFTER ************" << std::endl;
		// print_tree( node , "" );
		// std::cout << "***************************************" << std::endl;
		// rrt_publisher.publish( temp_rrt );
		// sleep(1.0);
		// std::cin.get();
		// std::cout << "after adding to tree" << std::endl;
		// n_near->children.push_back( new_node );
		
		// check if I am within delta range from the created node
		if( close_to_goal( x_new , goal_pt , delta , edge_distance ) ){
			// std::cout << "found to be clode to point " << std::endl;
			RRTNode* n_goal = new RRTNode( goal_pt , edge_distance , n_new );
			n_goal->is_goal = true;
			n_new->children.push_back( *n_goal );
			// std::cout << "added the goal to the tree with the is goal : " << n_new->children.back().is_goal <<  std::endl;
			return Reached;
		}
		else{
			return Advanced;
		}
	}
	return Trapped;
}


RRTNode
RRTPathPlanner::
build_rrt( const geometry_msgs::Point& x_init , const geometry_msgs::Point& goal_pt , const unsigned int& K , const double& bias){
	// initialize the tree
	RRTNode node( x_init );
	//randomize output from this point
	srand(time(NULL));

	//run the expreiment k times
	for( unsigned int i = 0; i < K; i++ ){
		geometry_msgs::Point x_rand = generate_random_point(goal_pt , bias);	
		if( extend_tree( node , x_rand , goal_pt , bias ) == Reached ){
			std::cout << "the number of nodes explored was : " << i << std::endl;
			return node;
		}
	}
	std::cout << "explored all possible " << K << " nodes " << std::endl;
	return node;
}

RRTPathPlanner::
RRTPathPlanner()
	:number_of_samples( 4000 ),
	 odom_recieved( false ),
	 map_recieved( false ),
	 obstacle_recieved( false ),
	 discretization( 0.1 ),
	 free_cell( log( 0.1 / ( 1.0 - 0.1 ) ) ),
	 delta( 0.5 ),
	 bias( 0.5 ),
	 neighboring( 0.7 )
{
}


RRTPathPlanner::
~RRTPathPlanner(){
}

void
RRTPathPlanner:: 
handle_odom( const nav_msgs::Odometry::ConstPtr& msg ){ 
	odom = *msg; 
	odom_recieved = true;
	return;
}

void 
RRTPathPlanner::
handle_goal( const geometry_msgs::Pose::ConstPtr& msg ){
	goal = *msg;
	if(!odom_recieved){
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = yaw_to_quaternion( 0.0 );
		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.linear.z = 0.0;
		odom.twist.twist.angular.x = 0.0;
		odom.twist.twist.angular.y = 0.0;
		odom.twist.twist.angular.z = 0.0;
	}
	search( odom.pose.pose , goal );
	return;
}

void
RRTPathPlanner::
handle_obstacles( const sim::RoundObstacles2D::ConstPtr& msg ){
	obstacles = *msg;
	if( !obstacle_recieved ){
		obstacle_recieved = true;
		projected_obstacles = obstacles;
		// update_obstacles_location( 0.5 );
	}
	// update_tree();
}


void
RRTPathPlanner::
handle_map( const map_msgs::OccupancyGridUpdate::ConstPtr& msg ){
	// std::cout << "map recieved with size : " << msg->data.size() << std::endl;
	map = *msg;
	map_recieved = true;
	return;
}

void
make_rrt_from_tree( path_planner::RRT& tree , RRTNode& node ){
	if( node.children.size() <= 0 ) return;
	for( unsigned int i = 0; i < node.children.size(); i++ ){
		tree.edges.push_back( path_planner::RRTEdge() );
		tree.edges.back().p1 = node.point;
		tree.edges.back().p2 = node.children[i].point;
		make_rrt_from_tree( tree , node.children[i] );
	}
	return;
}


/*
 * takes a start and a goal and ensures that the tree is built and
 * and that the path is constructed at all time
 */
void
RRTPathPlanner::
search(const geometry_msgs::Pose& start , const geometry_msgs::Pose& goal)
{
	root = build_rrt( start.position , goal.position , number_of_samples , bias);
	// std::cout << "built the RRT " << std::endl;

	make_rrt_from_tree( _tree , root );
	// std::cout << "just finished and trying to publish the tree" << std::endl;
	// print_tree( _tree );
	rrt_publisher.publish( _tree );
	// std::cout << "done publishing tree " << std::endl;
	path = construct_path( root );
	// std::cout << "publishing path of size: " << path.poses.size() << std::endl;
	path_publisher.publish( path );
	return;
}



/*
 * takes a vector of points and reverses it in a path 
 */
nav_msgs::Path
reverse_path( const std::vector<geometry_msgs::Point>& original , nav_msgs::Path& new_path){
	
	// new_path.poses.resize(original.size()+1);
	// std::cout << "new path new size : " << new_path.poses.size() << " while original size is : " << original.poses.size() << std::endl;
	for( unsigned int i = 0; i < original.size(); i++ ){
		geometry_msgs::Pose pose;
		pose.position = original[ original.size() - i -1];
		new_path.poses.push_back( pose_to_pose_stamped( pose ));
	}
	return new_path;
}


/*
 * search the tree to find a path 
 */
bool
search_tree( const RRTNode& node , std::vector< geometry_msgs::Point >& p , const geometry_msgs::Point& goal_pt , const double& delta){
	// std::cout << "searching for a path" << std::endl;
	// if the node is a leaf then we can add it or not
	if( node.children.size() <= 0 ){
		if( node.is_goal || compute_distance(node.point , goal_pt) <= delta){
			std::cout << " found the goal while node.is_goal says : " << node.is_goal <<  std::endl;
			p.push_back( node.point );
			return true;
		} 
		return false;
	}
	for( unsigned int i = 0; i < node.children.size(); i++ ){
		bool connected_to_goal = search_tree( node.children[i] , p , goal_pt , delta);
		if( connected_to_goal ){
			std::cout << "found a node leading to goal" << std::endl;
			p.push_back( node.children[i].point );
			return true;
		}
	}
	return false;
}

void
print_path( const nav_msgs::Path& p ){
	for( unsigned int i = 0; i < p.poses.size(); i++ ){
		std::cout << p.poses[i].pose.position << std::endl;
	}
}

nav_msgs::Path
RRTPathPlanner::
construct_path( const RRTNode& tree ){
	std::vector<geometry_msgs::Point> reversed_path;
	search_tree( tree , reversed_path , goal.position , delta);
	// nav_msgs::Path path;
	// std::cout << "got a closed list of size: " << closedlist.size() << std::endl;
	// nav_msgs::Path reversed_path = make_reversed_path( closedlist );
	nav_msgs::Path path;
	path.poses.push_back( pose_to_pose_stamped( odom.pose.pose ) );
	reverse_path( reversed_path , path);
	path.poses.push_back( pose_to_pose_stamped( goal ) );
	print_path( path );
	return path;
}

/*
 * compute the projected position of one of the obstacles
 */
void
project_obstacle( const sim::RoundObstacle2D& obs , sim::RoundObstacle2D& obs_proj, const double& t ){
	double r = obs.linear / obs.angular;
	double alpha = obs.angular * t;
	std::cout << "here" << std::endl;
	obs_proj.pose.x = obs.pose.x + ((-r * sin(obs.pose.theta)) + (r*(sin(obs.pose.theta + alpha))));
	std::cout << "here" << std::endl;
	obs_proj.pose.y = obs.pose.y + ((r * cos(obs.pose.theta)) - (r*(sin(obs.pose.theta + alpha))));
	std::cout << "here" << std::endl;
	obs_proj.pose.theta = alpha;
	std::cout << "here" << std::endl;
	return;
}

/*
 * update the position of the occupied spots
 */
void
RRTPathPlanner::
update_obstacles_location( const double t ){
	if( t == 0.0 ) return;
	if( obstacles.obstacles.size() <= 0 ) return;
	for(unsigned int i = 0; i < obstacles.obstacles.size(); i++){
		project_obstacle( obstacles.obstacles[i] , projected_obstacles.obstacles[i] , t);
	}
	projected_obstacle_publisher.publish( projected_obstacles );
}

/*
 * ensured that the tree is not in the way of any obstacles
 */
void
RRTPathPlanner::
update_tree(){
	// trim_conflicts();
	// if( !path_exists() ){
		// replan();
		// find_path();
	// }

}	
