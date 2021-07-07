#ifndef PATH_PLANNER_DLITE_H_
#define PATH_PLANNER_DLITE_H_
#include <carmen/carmen.h>
#include <carmen/rddf_interface.h>
#include <carmen/obstacle_distance_mapper_interface.h>
#include <carmen/grid_mapping.h>
#include <carmen/collision_detection.h>
#include <carmen/rddf_interface.h>
#include <carmen/rddf_messages.h>
#include <carmen/offroad_planner.h>
#include <carmen/route_planner_interface.h>
#include <carmen/global_graphics.h>
#include <carmen/mapper_interface.h>
#include <algorithm>
#include <car_model.h>
#include <float.h>
#include <math.h>
#include <queue>
#include <list>
#include <vector>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <sys/time.h>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_math.h>

#define OBSTACLE_DISTANCE_MIN 0.2
#define EXPAND_NODES_V 1.42
#define PENALTIES_W1 2.0
#define PENALTIES_W2 5.0
#define SEND_MESSAGE_IN_PARTS 0
#define USE_DNO 0
#define USE_SMOOTH 1
#define SMOOTHNESS_WEIGHT 30.0
#define OBSTACLE_WEIGHT 10.0
#define CURVATURE_WEIGHT 0.0
#define DELTA_T 0.01                      // Size of step for the ackerman Euler method
#define LANE_WIDTH 	2.4
#define NUM_LANES	1
#define DELTA2D(x1,x2) ((carmen_robot_and_trailer_traj_point_t){x1.x - x2.x, x1.y - x2.y, 0.0, 0.0, 0.0})
#define HEAP_USED 2 // 0 = fibonacci heap; 1 = priority queue; 2 = binary heap;
#define DRAW_EXPANSION_TREE 0
#define RUN_EXPERIMENT 0
#define COMPARE_HEURISTIC 0


enum possible_states {Not_visited, Open, Closed};
enum motion_direction {Forward, Backward};
Key k_old_{0, 0};
Node start_node, goal_node, last_node;
double k_m_ = 0;
std::vector<std::vector<double>> rhs_;
std::vector<std::vector<double>> g_;
#if HEAP_USED == 0
	printf("Usando Fibonacci Heap\n");
	boost::heap::fibonacci_heap<state_node*, boost::heap::compare<StateNodePtrComparator>> FH;
    #elif HEAP_USED == 1
	printf("Usando Priority queue\n");
	std::priority_queue<state_node*, std::vector<state_node*>, StateNodePtrComparator> FH;
    #elif HEAP_USED == 2
	printf("Usando Binary heap\n");
    boost::heap::d_ary_heap<state_node*, boost::heap::arity<boost_arity>, boost::heap::compare<StateNodePtrComparator>> FH;
    #endif

//Se COMPARE_HEURISTIC == 1, o código irá usar o heuristic_number para alternar as heurísticas e vai rodar a busca 4 vezes.

static const char* heuristic_compare_message[] = {"Máximo das duas heurísticas ------------------------------",
											"Somente Heurística com obstáculo (GDM) -------------------",
											"Somente Heurística sem obstáculos (NHCM)------------------",
											"Distância Euclidiana como Heurística ---------------------"};

typedef struct {
    double state_map_resolution;
    int state_map_theta_resolution;
    int precomputed_cost_size;
    int precomputed_cost_theta_size;
    double precomputed_cost_resolution;
    char *precomputed_cost_file_name;
    int use_matrix_cost_heuristic;

  } carmen_path_planner_dlite_t;

typedef struct pose_node
{
	double x;
	double y;
	double theta;
	motion_direction r;
} pose_node, *pose_node_p;

struct Key {
  	double first;
  	double second;
	bool operator<(const Key& k) const {
		return first < k.first || (first == k.first && second < k.second);
	}
	bool operator>(const Key& k) const {
    	return first > k.first || (first == k.first && second > k.second);
	}	
	bool operator==(const Key& k) const {
    	return first == k.first && second == k.second;
  	}
	bool operator!=(const Key& k) const {
    	return !(first == k.first && second == k.second);
  	}
};

typedef struct Node
{
	pose_node pose;
	double h;
	double c;
	Node *parent;
	int expanded_nodes;
};

struct NodeKeyPair {
  Node node;
  Key key;
}

typedef struct grid_state
{
	possible_states state;
	double g;
} grid_state, *grid_state_p;


typedef struct nonholonomic_heuristic_cost
{
	double h;
} nonholonomic_heuristic_cost, *nonholonomic_heuristic_cost_p;


typedef struct param_otimization
{
	carmen_robot_and_trailer_traj_point_t *points;
	int *anchor_points;
	int path_size;
	int problem_size;
} param_t, *param_p;


class StateNodePtrComparator {
public:
	bool operator() (state_node *a, state_node *b) const
	{
		return (a->f > b->f);
	}
};

std::vector<Node> GetMotion() {
  return {
    Node(0, 1, 1, 0, 0, 0),
    Node(1, 0, 1, 0, 0, 0),
    Node(0, -1, 1, 0, 0, 0),
    Node(-1, 0, 1, 0, 0, 0)
    // Node(1, 1, sqrt(2), 0, 0, 0),
    // Node(1, -1, sqrt(2), 0, 0, 0),
    // Node(-1, 1, sqrt(2), 0, 0, 0),
    // Node(-1, -1, sqrt(2), 0, 0, 0)
  };
  // NOTE: Add diagonal movements for A* and D* only after the heuristics in the
  // algorithms have been modified. Refer to README.md. The heuristics currently
  // implemented are based on Manhattan distance and dwill not account for
  // diagonal/ any other motions
}

//Reed Shepp ////////////////////////////

typedef enum {
	RS_TURN_RIGHT, RS_TURN_LEFT, RS_STRAIGHT, RS_FWD, RS_BWD, RS_NONE
} RS_POSSIBLE_MOVES;

typedef struct
{
	int turn;
	int move;
} rs_move;

rs_move*
rs_get_moves(int numero);

int
fct_curve(int ty, int orientation, double val, carmen_robot_and_trailer_traj_point_t * start, double delta, carmen_robot_and_trailer_traj_point_t * points, int n);

void
rs_init_parameters(double max_phi, double distance_between_front_and_rear_axles);

double
reed_shepp(carmen_robot_and_trailer_traj_point_t start, carmen_robot_and_trailer_traj_point_t goal, int *numero, double *tr, double *ur, double *vr);

int
constRS(int num, double t, double u, double v, carmen_robot_and_trailer_traj_point_t start, carmen_robot_and_trailer_traj_point_t * points);

int
get_index_of_nearest_pose_in_path(carmen_robot_and_trailer_traj_point_t *path, carmen_point_t globalpos, int path_length);

carmen_robot_and_trailer_traj_point_t *
get_poses_back(carmen_robot_and_trailer_traj_point_t *path, int nearest_pose_index);

void
add_lanes(carmen_route_planner_road_network_message &route_planner_road_network_message,
		carmen_robot_and_trailer_traj_point_t *path_copy);

void
free_lanes(carmen_route_planner_road_network_message route_planner_road_network_message);

double *
get_goal_distance_map(carmen_point_t goal_pose, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map);

std::vector<carmen_robot_and_trailer_traj_point_t>
carmen_path_planner_dlite_search(pose_node *initial_pose, pose_node *goal_pose,
		carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double *goal_distance_map,
		nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map);

int
smooth_rddf_using_conjugate_gradient(std::vector<carmen_robot_and_trailer_traj_point_t> &astar_path);

offroad_planner_plan_t
dlite_mount_offroad_planner_plan(carmen_point_t *robot_pose, carmen_robot_and_trailer_pose_t *goal_pose, std::vector<carmen_robot_and_trailer_traj_point_t> path_result);

carmen_map_t *
copy_grid_mapping_to_map(carmen_map_t *map, carmen_mapper_map_message *grid_map);

void
alloc_cost_map();

double
carmen_compute_abs_angular_distance(double theta_1, double theta_2);

void
override_initial_and_goal_poses(carmen_point_t &initial_pose, carmen_point_t &goal_pose);

#endif /* PATH_PLANNER_DLITE_H__ */
