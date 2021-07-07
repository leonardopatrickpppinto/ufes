#include "path_planner_dlite.h"

extern carmen_robot_ackerman_config_t robot_config;
extern carmen_path_planner_dlite_t dlite_config;
carmen_semi_trailer_config_t semi_trailer_config;

extern carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map;
extern carmen_map_p map_occupancy;

extern nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map;
extern int use_nonholonomic_heuristic_cost_map;
extern int heuristic_number;

extern int grid_state_map_x_size;
extern int grid_state_map_y_size;

cv::Mat map_image;
int expanded_nodes_by_dlite;
const int boost_arity = 2;

char expansion_tree_file_name[] = "Expansion_illustration_0.png";

//---------------------Funções Auxiliares------------------------------------
static int
sign(double a)
{
	if (a >= 0.0)
		return 1;
	else
		return -1;
}

grid_state_p ****
alloc_grid_state_map()
{
	grid_state_p ****grid_state_map;
	grid_state_map_x_size = round((obstacle_distance_grid_map->config.x_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);
	grid_state_map_y_size = round((obstacle_distance_grid_map->config.y_size * obstacle_distance_grid_map->config.resolution) / astar_config.state_map_resolution);
	printf("Distance map origin: %f %f\n", obstacle_distance_grid_map->config.x_origin, obstacle_distance_grid_map->config.y_origin);
	int theta_size = astar_config.state_map_theta_resolution;
	grid_state_map = (grid_state_p ****)calloc(grid_state_map_x_size, sizeof(grid_state_p ***));
	carmen_test_alloc(grid_state_map);

	for (int i = 0; i < grid_state_map_x_size; i++)
	{
		grid_state_map[i] = (grid_state_p ***)calloc(grid_state_map_y_size, sizeof(grid_state_p **));
		carmen_test_alloc(grid_state_map[i]);

		for (int j = 0; j < grid_state_map_y_size; j++)
		{
			grid_state_map[i][j] = (grid_state_p **)calloc(theta_size, sizeof(grid_state_p *));
			carmen_test_alloc(grid_state_map[i][j]);

			for (int k = 0; k < theta_size; k++)
			{
				grid_state_map[i][j][k] = (grid_state_p *)calloc(2, sizeof(grid_state_p));
				carmen_test_alloc(grid_state_map[i][j][k]);

				for (int l = 0; l < 2; l++)
				{
					grid_state_map[i][j][k][l] = (grid_state_p)malloc(sizeof(grid_state));
					carmen_test_alloc(grid_state_map[i][j][k][l]);

					grid_state_map[i][j][k][l]->state = Not_visited;
					grid_state_map[i][j][k][l]->g = 0;
				}
			}
		}
	}
	return grid_state_map;
}

Node *
new_state_node(double pose_x, double pose_y, double pose_theta, motion_direction pose_r, double g, double h, double f, double rhs, state_node *parent)
{
	Node *new_state = (state_node *)malloc(sizeof(state_node));

	new_state->pose.x = pose_x;
	new_state->pose.y = pose_y;
	new_state->pose.theta = pose_theta;
	new_state->pose.r = pose_r;
	new_state->g = g;
	new_state->h = h;
	new_state->f = f;
	new_state->rhs = rhs;
	new_state->parent = parent;
	new_state->expanded_nodes = 0;
	return (new_state);
}

void check_initial_nodes(state_node *initial_node, state_node *goal_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, int &failed)
{
	failed = 0;

	carmen_robot_and_trailer_traj_point_t trajectory_pose;
	trajectory_pose.x = initial_node->pose.x;
	trajectory_pose.y = initial_node->pose.y;
	trajectory_pose.theta = initial_node->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;
	printf("Initial_pose = %f %f %f\n", initial_node->pose.x, initial_node->pose.y, initial_node->pose.theta);

	if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map) < OBSTACLE_DISTANCE_MIN)
	{
		printf("!!!!!!!!!!!!!!!!!!!!!!! Robot_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map));
		failed = 1;
	}

	trajectory_pose.x = goal_node->pose.x;
	trajectory_pose.y = goal_node->pose.y;
	trajectory_pose.theta = goal_node->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;

	if (carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map) < OBSTACLE_DISTANCE_MIN)
	{
		printf("!!!!!!!!!!!!!!!!!!!!!!! Goal_pose next to obstacle: %f\n", carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map));
		failed = 1;
	}
	printf("Goal_pose = %f %f %f\n", goal_node->pose.x, goal_node->pose.y, goal_node->pose.theta);
}

void get_current_pos(Node *current_node, int &x, int &y, int &theta, int &direction, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	x = get_grid_state_map_x(current_node->pose.x, obstacle_distance_grid_map);
	y = get_grid_state_map_y(current_node->pose.y, obstacle_distance_grid_map);
	theta = get_dlite_map_theta(current_node->pose.theta, astar_config.state_map_theta_resolution);
	direction = current_node->pose.r;
}

int is_goal(state_node **current_node, state_node *goal_node, int cont_rs_nodes)
{
	if (DIST2D((*current_node)->pose, goal_node->pose) < 0.5 && (abs(carmen_radians_to_degrees((*current_node)->pose.theta) - carmen_radians_to_degrees(goal_node->pose.theta)) < 5))
		return 1;

	else if (cont_rs_nodes % int((*current_node)->h + 1) == 0)
	{
		analytic_expansion(current_node, goal_node, obstacle_distance_grid_map);

		if (DIST2D((*current_node)->pose, goal_node->pose) < 0.5 && (abs(carmen_radians_to_degrees((*current_node)->pose.theta) - carmen_radians_to_degrees(goal_node->pose.theta)) < 5))
			return 1;
	}

	return 0;
}

std::vector<Node>
expand_node(Node *n, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	std::vector<Node> neighbors;
	double target_phi[3] = {-robot_config.max_phi, 0.0, robot_config.max_phi};
	double target_v[2] = {EXPAND_NODES_V, -EXPAND_NODES_V};

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Node new_state = (Node)malloc(sizeof(Node));
			carmen_test_alloc(new_state);
			new_state->pose = carmen_path_planner_astar_ackerman_kinematic(n->pose, robot_config.distance_between_front_and_rear_axles, target_phi[j], target_v[i]);
			if (target_v[i] < 0)
			{
				new_state->pose.r = Backward;
			}
			else
			{
				new_state->pose.r = Forward;
			}

			if (is_valid_state(new_state, obstacle_distance_grid_map) == 1)
			{
				neighbors.push_back(new_state);
			}
			else
				free(new_state);
		}
	}
	return neighbors;
}
////////////////////////////////////////////////
double
movement_cost(state_node *current_node, state_node *new_node)
{
	return DIST2D(current_node->pose, new_node->pose);
}

double
penalties(state_node *current_node, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map)
{
	carmen_robot_and_trailer_traj_point_t trajectory_pose;
	trajectory_pose.x = current_node->pose.x;
	trajectory_pose.y = current_node->pose.y;
	trajectory_pose.theta = current_node->pose.theta;
	trajectory_pose.v = 0;
	trajectory_pose.phi = 0;

	double distance_to_nearest_obstacle = 0;
	if (USE_DNO)
		distance_to_nearest_obstacle = 1 / carmen_obstacle_avoider_car_distance_to_nearest_obstacle(trajectory_pose, obstacle_distance_grid_map);

	return distance_to_nearest_obstacle +
		   PENALTIES_W1 * current_node->pose.r * movement_cost(current_node, current_node->parent) +
		   PENALTIES_W2 * (current_node->pose.r != current_node->parent->pose.r);
}

double
h(state_node *current_pose, state_node *goal_pose, grid_state_p ****grid_state_map, carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double *goal_distance_map, nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map)
{
	double ho = -1;
	double nh = -1;

	int x_c;
	int y_c;
	int theta_c;
	int direction_c;
	get_current_pos(current_pose, x_c, y_c, theta_c, direction_c, obstacle_distance_grid_map);

	ho = goal_distance_map[y_c + x_c * grid_state_map_y_size];

	if (astar_config.use_matrix_cost_heuristic && use_nonholonomic_heuristic_cost_map)
	{
		int x = ((current_pose->pose.x - goal_pose->pose.x) * cos(current_pose->pose.theta) - (current_pose->pose.y - goal_pose->pose.y) * sin(current_pose->pose.theta)) / astar_config.precomputed_cost_resolution;
		int y = ((current_pose->pose.x - goal_pose->pose.x) * sin(current_pose->pose.theta) + (current_pose->pose.y - goal_pose->pose.y) * cos(current_pose->pose.theta)) / astar_config.precomputed_cost_resolution;
		int theta;

		if ((x <= 0 && y >= 0) || (x >= 0 && y <= 0))
			theta = get_astar_map_theta(carmen_normalize_theta(-(goal_pose->pose.theta - current_pose->pose.theta)), astar_config.precomputed_cost_theta_size);
		else
			theta = get_astar_map_theta(carmen_normalize_theta(goal_pose->pose.theta - current_pose->pose.theta), astar_config.precomputed_cost_theta_size);

		int half_map = round(((astar_config.precomputed_cost_size) / astar_config.precomputed_cost_resolution) / 2);

		if (x < half_map && y < half_map && x > -half_map && y > -half_map)
		{
			nh = nonholonomic_heuristic_cost_map[x + half_map][y + half_map][theta]->h;
		}
	}

	//	printf("NH = %f HO = %f\n", nh, ho);

	double return_h;

#if COMPARE_HEURISTIC

	switch (heuristic_number)
	{
	case 0:
		return_h = std::max(nh, ho);
		break;
	case 1:
		return_h = ho;
		break;
	case 2:
		return_h = std::max(nh, DIST2D(current_pose->pose, goal_pose->pose));
		break;
	case 3:
		return_h = DIST2D(current_pose->pose, goal_pose->pose);
		break;
	}

#else
	return_h = std::max(nh, ho);
#endif

	return return_h;
}

///////////////////////////////////////////////

std::vector<Node> GetNeighbours(Node &u)
{
	std::vector<Node> neighbours;
	neighbours = expand_node(u, obstacle_distance_grid_map);
	return neighbours;
}

std::vector<Node> GetPred(Node &u)
{
	return GetNeighbours(u);
}

std::vector<Node> GetSucc(Node &u)
{
	return GetNeighbours(u);
}

Key CalculateKey(Node &s) const
{
	return Key{std::min(g_[s->pose->x_][s->pose->y_], rhs_[s->pose->x_][s->pose->y_]) + H(start_, s) + k_m_,
			   std::min(g_[s->pose->x_][s->pose->y_], rhs_[s->pose->x_][s->pose->y_])};
}

std::vector<std::vector<double>> CreateGrid()
{
	return std::vector<std::vector<double>>(
		n_, std::vector<double>(n_, std::numeric_limits<double>::max()));
}

void Initialize()
{
	FH.clear();
	k_m_ = 0;
	rhs_ = CreateGrid();
	g_ = CreateGrid();
	int x = goal_node->pose->x;
	int y = goal_node->pose->y;
	rhs_[x][y] = 0;
	FH.insert(NodeKeyPair{goal_node, CalculateKey(goal_node)});
}

void ComputeShortestPath()
{
	while ((!FH.empty() && FH.top()->key < CalculateKey(start_node)) ||
		   (rhs_[start_node->pose->x][start_node->pose->y] != g_[start_node->pose->x][start_node->pose->y]))
	{
		k_old_ = FH.top()->key;
		Node u = FH.top()->node;
		FH.pop();
		Key u_key = CalculateKey(u);
		if (k_old_ < u_key)
		{
			FH.insert(NodeKeyPair{u, u_key});
		}
		else if (g_[u->pose->x][u->pose->y] > rhs_[u->pose->x][u->pose->y])
		{
			g_[u->pose->x][u->pose->y] = rhs_[u->pose->x][u->pose->y];
			std::vector<Node> N = GetPred(u);
			for (int i = 0; i < N.size(); i++)
			{
				UpdateVertex(N[i]);
			}
		}
		else
		{
			g_[u->pose->x][u->pose->y] = std::numeric_limits<double>::max();
			std::vector<Node> N = GetPred(u);
			for (int i = 0; i < N.size(); i++)
			{
				UpdateVertex(N[i]);
			}
			//UpdateVertex(u);
		}
	}
}

void UpdateVertex(Node &u)
{
	if (grid_state_map[u->pose->x][u->pose->y] == 0)
	{
		grid_state_map[u->pose->x][u->pose->y] = 2;
	}
	if (!is_goal(u, goal_node))
	{
		rhs_[u->pose->x][u->pose->y] = std::numeric_limits<double>::max();
		std::vector<Node> N = GetSucc(u);
		for (int i = 0; i < N.size(); i++)
		{
			rhs_[u->pose->x][u->pose->y] =
				std::min(rhs_[u->pose->x][u->pose->y], C(u, N[i]) + g_[N[i]->pose->x][N[i]->pose->y]);
		}
	}
	if (FH.isElementInStruct({u, {}}))
	{
		FH.remove(NodeKeyPair{u, Key()});
	}
	if (rhs_[u->pose->x][u->pose->y] != g_[u->pose->x][u->pose->y])
	{
		FH.insert(NodeKeyPair{u, CalculateKey(u)});
	}
}

std::vector<Node> DStarLite::DetectChanges()
{
	std::vector<Node> obstacles;
	if (time_discovered_obstacles_.find(time_step_) !=
		time_discovered_obstacles_.end())
	{
		const auto discovered_obstacles_at_time =
			time_discovered_obstacles_[time_step_];
		for (const auto &discovered_obstacle_at_time :
			 discovered_obstacles_at_time)
		{
			if (!((start_node->pose->x == discovered_obstacle_at_time.x_ &&
				   start_node->pose->y == discovered_obstacle_at_time.y_) ||
				  (goal_node->pose->x == discovered_obstacle_at_time.x_ &&
				   goal_node->pose->y == discovered_obstacle_at_time.y_)))
			{
				grid_[discovered_obstacle_at_time.x_][discovered_obstacle_at_time.y_] =
					1;
				obstacles.push_back(discovered_obstacle_at_time);
			}
		}
	}
	if (create_random_obstacles_ && rand() > 1.0 / static_cast<double>(n_))
	{
		const int x = rand() % n_;
		const int y = rand() % n_;
		if (!((start_.x_ == x && start_.y_ == y) ||
			  (goal_.x_ == x && goal_.y_ == y)))
		{
			grid_[x][y] = 1;
			obstacles.emplace_back(Node(x, y));
		}
	}
	return obstacles;
}

//---------------------Função principal--------------------------------------
std::vector<carmen_robot_and_trailer_traj_point_t>
carmen_path_planner_dlite_search(pose_node *initial_pose, pose_node *goal_pose,
								 carmen_obstacle_distance_mapper_map_message *obstacle_distance_grid_map, double *goal_distance_map,
								 nonholonomic_heuristic_cost_p ***nonholonomic_heuristic_cost_map)
{
	std::vector<Node> path_result;

	grid_state_p ****grid_state_map = alloc_grid_state_map();
	start_node = new_state_node(initial_pose->x, initial_pose->y, initial_pose->theta, initial_pose->r, 0, 0, 0, NULL);
	goal_node = new_state_node(goal_pose->x, goal_pose->y, goal_pose->theta, goal_pose->r, 0, 0, 0, NULL);

	int initial_nodes_failed;
	check_initial_nodes(start_node, goal_node, obstacle_distance_grid_map, initial_nodes_failed);
	if (initial_nodes_failed == 1)
		return path_result;

	get_current_pos(start_node, x, y, theta, direction, obstacle_distance_grid_map);
	path_result.pushBack(start_node);
	grid_state_map[x][y][theta][direction]->state = Open;
	//Dúvida
	grid_state_map[x][y][theta][direction]->g = 4;
	last_node = start_node;
	Initialize();
	ComputeShortestPath();
	//////
	while (!is_goal(start_node, goal_node))
	{
		time_step_++;
		if (g_[start_node->pose->x][start_node->poe->y] == std::numeric_limits<double>::max())
		{
			path_result.clear();
			path_result.push_back(start_node);
			return {false, path_result};
		}
		std::vector<Node> successors = GetSucc(start_node);

		//Dúvda
		grid_[start_.x_][start_.y_] = 3;
		start_node = *std::min_element(std::begin(successors), std::end(successors),
									   [this](const auto &n1, const auto &n2)
									   {
										   return C(start_, n1) + g_[n1.x_][n1.y_] <
												  C(start_, n2) + g_[n2.x_][n2.y_];
									   });
		path_result.push_back(start_node);
		//Dúvida
		grid_[start_node->pose->x][start_node->pose->y] = 4;

		if (const auto changed_nodes = DetectChanges(); !changed_nodes.empty())
		{
			k_m_ += H(last, start_);
			last = start;
			for (const auto node : changed_nodes)
			{
				UpdateVertex(node);
			}
			ComputeShortestPath();
		}
	}
	path[0].id_ = path[0].x_ * n_ + path[0].y_;
	path[0].pid_ = path[0].id_;
	path[0].cost_ = 0;
	for (int i = 1; i < path.size(); i++)
	{
		path[i].id_ = path[i].x_ * n_ + path[i].y_;
		const auto delta =
			Node(path[i].x_ - path[i - 1].x_, path[i].y_ - path[i - 1].y_);
		path[i].cost_ = path[i - 1].cost_ +
						std::find_if(std::begin(motions_), std::end(motions_),
									 [&delta](const Node &motion)
									 {
										 return CompareCoordinates(motion, delta);
									 })
							->cost_;
		path[i].pid_ = path[i - 1].id_;
	}
	return {true, path};
}
