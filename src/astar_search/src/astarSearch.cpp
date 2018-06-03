//Filename astarSearch

#include "ros/ros.h"
#include "astar_search/aStarClass.h"
#include "astar_search/discrete_2D_rigid_body.h"

#include <fstream>
#include <string>
#include <math.h>
#include <memory>
#include <boost/functional/hash.hpp>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/String.h"

#include "pathfinding_msgs/State.h"
#include "pathfinding_msgs/Path.h"
#include "pathfinding_msgs/PathRequest.h"

namespace state_search{ namespace astar_search {

using state_search::State;

const int ROBOT_LENGTH = 15;
const int ROBOT_WIDTH = 11;
Discrete2DRigidBody rigid_body(ROBOT_LENGTH, ROBOT_WIDTH, 0.0);

bool isValidState(int, int, float);

struct SE2State : public state_search::State
{
private:
  int x_, y_;
  float yaw_;
    
public:
  SE2State(int x, int y, float yaw):x_(x), y_(y), yaw_(yaw) {};
  int getX() const {return x_;};
  int getY() const {return y_;};
  float getYaw() const {return yaw_;};

  std::size_t hash()
  {
    std::size_t seed = 0;
    boost::hash_combine(seed, x_);
    boost::hash_combine(seed, y_);
    return seed;
  }

  inline bool operator==(const std::shared_ptr<State> other)
  {
    std::shared_ptr<const SE2State> o = std::static_pointer_cast<const SE2State>(other);
    return x_ == o->getX() && y_ == o->getY();

  } 

  inline bool operator<(const std::shared_ptr<State> other)
  {
    std::shared_ptr<const SE2State> o = std::static_pointer_cast<const SE2State>(other);
    if(x_ != o->getX())
      return x_ < o->getX();
    if(y_ != o->getY())
      return y_ < o->getY();
    return true;
  }
};

float manhattanDistanceHeuristic(SE2State& current_state, SE2State& goal_state)
{
    return abs(goal_state.getX() - current_state.getX())
         + abs(goal_state.getY() - current_state.getY());
}

float euclidianDistanceHeuristic(SE2State& current_state, SE2State& goal_state)
{
  return sqrt(pow(goal_state.getX() - current_state.getX(), 2)
            + pow(goal_state.getY() - current_state.getY(), 2)
  );
}

/**
 * A class that defines a search problem using a start and goal state.
 */
class SearchProblem : public ProblemDefinition
{ 
public:
  static const std::vector< std::vector< std::pair<int, int> > > RIGID_BODIES;

  /**
   * Constructs a search problem using a start and goal state
   *
   * @param start_state Shared pointer to problem's start state.
   * @param goal_state  Shared pointer to problem's goal state.
  */
  SearchProblem(std::shared_ptr<SE2State> start_state, std::shared_ptr<SE2State> goal_state)
  {
    start_state_ = std::static_pointer_cast<State>(start_state);
    goal_state_ = std::static_pointer_cast<State>(goal_state);
  }

  /**
   * Start state getter.
   *
   * @return Shared pointer to the start state.
   */
  std::shared_ptr<State> getStartState(){return start_state_;}

  /**
   * Checks whether a given state is the same as the goal state.
   * 
   * @param current_state Pointer to the state being checked.
   *
   * @return True if the states are equal, false otherwise.
   */
  bool isGoalState(std::shared_ptr<State> current_state){return *current_state == goal_state_;}

  /**
   *
   * @return Heuristic value of the given state
   */
  float getHeuristicValue(std::shared_ptr<State> current_state)
  {
    std::shared_ptr<SE2State> current_se2state = std::dynamic_pointer_cast<SE2State>(current_state);
    std::shared_ptr<SE2State> goal_se2state = std::dynamic_pointer_cast<SE2State>(goal_state_);
    return manhattanDistanceHeuristic(*current_se2state, *goal_se2state);
  }

  /**
   * Generates valid successor states of a given SE2State.
   */
  std::vector< std::pair<std::shared_ptr<State>, float > > getSuccessorStates(std::shared_ptr<State> current_state)
  {
    std::shared_ptr<SE2State> current_se2state = std::dynamic_pointer_cast<SE2State>(current_state);
    std::vector<std::pair<std::shared_ptr<State>, float> > successor_states;

    for(auto action : LEGAL_MOVES)
    {
      int next_x = current_se2state->getX() + action.getX();
      int next_y = current_se2state->getY() + action.getY();
      int next_theta = current_se2state->getYaw() + action.getYaw();
      if(isValidState(next_x, next_y, next_theta))
      {
        float cost = sqrt((float)(pow(action.getX(),2) + pow(action.getY(),2))) + abs(action.getYaw()) * 3.14 / 180 * ROBOT_LENGTH / 2;
        std::pair<std::shared_ptr<State>, float> successor_pair(
          std::static_pointer_cast<State>(
            std::make_shared<SE2State>(next_x, next_y, next_theta)
          ),
          cost
        );
        successor_states.push_back(successor_pair);
      }
    }
    return successor_states;
  }

private:
  static const std::vector<SE2State> LEGAL_MOVES;
};

const std::vector<SE2State> SearchProblem::LEGAL_MOVES = {
  SE2State(1, 0, 0.0),
  SE2State(-1, 0, 0.0),
  SE2State(0, 1, 0.0),
  SE2State(0, -1, 0.0),
  SE2State(1, 1, 0.0),
  SE2State(-1, -1, 0.0),
  SE2State(-1, 1, 0.0),
  SE2State(1, -1, 0.0),
  SE2State(0, 0, 45.0),
  SE2State(0, 0, -45.0)
};

const std::vector<
  std::vector<
    std::pair<int, int>
  >
> SearchProblem::RIGID_BODIES = {
  rigid_body.getRigidBodyAtAngle(0),
  rigid_body.getRigidBodyAtAngle(45),
  rigid_body.getRigidBodyAtAngle(90),
  rigid_body.getRigidBodyAtAngle(135),
  rigid_body.getRigidBodyAtAngle(180),
  rigid_body.getRigidBodyAtAngle(225),
  rigid_body.getRigidBodyAtAngle(270),
  rigid_body.getRigidBodyAtAngle(315)
};

unsigned int map_width, map_height;
float map_resolution;
unsigned char* map;
const int MAP_STATE_FREE_THRESHOLD = 10;

bool isValidState(int x, int y, float angle)
{
  int angle_quant = (int(angle / 45.0) % 8 + 8) % 8;
  for(auto point : SearchProblem::RIGID_BODIES[angle_quant])
  {
    int i = point.first;
    int j = point.second;
    bool is_x_out_of_bounds = x + i < 0 || x + i >= map_width;
    bool is_y_out_of_bounds = y + j < 0 || y + j >= map_height;
    if(is_x_out_of_bounds || is_y_out_of_bounds)
    {
      return false;
    }
    char current_cell = map[(y + j) * map_width + x + i];
    bool is_state_free = current_cell > MAP_STATE_FREE_THRESHOLD;
    if(is_state_free)
    {
      return false;
    }
  }
  return true;
}

/** 
 * A service callback function that initiates the search proces based on
 * received start and goal state. The response contains either a path between
 * start and goal state or an error message.
 * @param req Part of service message that contains parameters needed to
 *            define the search problem.
 * @param res Part of service message where the response is stored.
 *
 * @return Always true so the response can be sent.
 */
bool requestSearch(pathfinding_msgs::PathRequest::Request &req,
                   pathfinding_msgs::PathRequest::Response &res)
{
  //Create shared_ptrs to start and goal states and define the search problem
  std::shared_ptr<SE2State> start_state = std::make_shared<SE2State>(
    (int)req.startState.x, 
    (int)req.startState.y, 
    (float)req.startState.theta
  );
  std::shared_ptr<SE2State> goal_state = std::make_shared<SE2State>(
    (int)req.goalState.x, 
    (int)req.goalState.y, 
    (float)req.goalState.theta
  );
  SearchProblem problem = SearchProblem(start_state, goal_state);

  state_search::astar_class::AStar astar(&problem);

  double beginning_time = ros::Time::now().toSec(); //note the time the search began

  std::vector<std::shared_ptr<State>> path = astar.search();

  if(astar.isPathFound())
  {
    //output information about the search
    float required_time = ros::Time::now().toSec() - beginning_time;
    ROS_INFO("A* found a path in %f", required_time);
    ROS_INFO("Path length is %ld", path.size());
    ROS_INFO("Total node expanded count is %d", astar.getOpenedNodesCount());
    float path_len;
  
    //Copy found path to service response message
    for(int i = 0; i < path.size(); i++)
    {
      auto se2state = std::dynamic_pointer_cast<SE2State>(path[i]);
      pathfinding_msgs::State state;

      state.x = se2state->getX();
      state.y = se2state->getY();
      state.theta = se2state->getYaw();
      res.path.state.push_back(state);
      if(i!=0)
      {
        auto se2state_old = std::dynamic_pointer_cast<SE2State>(path[i-1]);
        float x_diff = (se2state->getX() - se2state_old->getX()) * map_resolution;
        float y_diff = (se2state->getY() - se2state_old->getY()) * map_resolution;
        path_len += sqrt(x_diff * x_diff + y_diff * y_diff);
      }
    }
    ROS_INFO("Path length = %f", path_len);
  }
  else 
  {
    ROS_INFO("Path couldn't be found");
    res.error = "Path not found";
  }
  return true;
}

void mapReceiver(const nav_msgs::OccupancyGrid mapData)
{

  if(map != NULL)
  {
    free(map);
  }
  
  map_width = mapData.info.width;
  map_height = mapData.info.height;
  map_resolution = mapData.info.resolution;
  map = (unsigned char*) malloc(map_width*map_height);

  for(int i=0; i < mapData.info.height; i++)
  {
    for(int j=0; j < mapData.info.width; j++)
    {
      int index = (map_width * i) + j;
      map[index] = static_cast<unsigned char>(mapData.data[index]);
    }
  }
}

} } //end namespace state_search::astar_search

/*!
 * Main program.
 * Initilizes ros environment, starts a subscription and service, then loops forever and ever.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "astarSearch");
    ros::NodeHandle n;

    ros::Subscriber mapSub = n.subscribe("map", 10, state_search::astar_search::mapReceiver);
    ros::ServiceServer service = n.advertiseService("astarSearch", state_search::astar_search::requestSearch);


    ros::spin();
    return 0;
}
