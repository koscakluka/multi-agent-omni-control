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

namespace astar_search
{

bool isFree(int, int);

struct SE2State : public State
{
private:
  int x_, y_;
  float yaw_;
    
public:
  SE2State(int x, int y, float yaw):x_(x), y_(y), yaw_(yaw) {};
  int getX() const {return x_;};
  int getY() const {return y_;};
  float getYaw() const {return yaw_;};
  void print() const {ROS_INFO("%d %d", x_, y_);};

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

inline bool operator==(const SE2State& lhs, const SE2State& rhs)
{
  return lhs.getX() == rhs.getX() && lhs.getY() == rhs.getY();
}

float manhattanDistanceHeuristic(SE2State& current_state, SE2State& goal_state)
{
    return abs(goal_state.getX() - current_state.getX()) + abs(goal_state.getY() - current_state.getY());
}

float euclidianDistanceHeuristic(SE2State& current_state, SE2State& goal_state)
{
  return sqrt(pow(goal_state.getX() - current_state.getX(), 2) + pow(goal_state.getX() - current_state.getX(), 2));
}

class SearchProblem : public ProblemDefinition
{
public:
  SearchProblem(std::shared_ptr<SE2State> start_state, std::shared_ptr<SE2State> goal_state)
  {
    start_state_ = std::static_pointer_cast<State>(start_state);
    goal_state_ = std::static_pointer_cast<State>(goal_state);
  }

  std::shared_ptr<State> getStartState(){
    return start_state_;
  }

  bool isGoalState(std::shared_ptr<State> current_state)
  { 
    std::shared_ptr<SE2State> current_se2state = std::static_pointer_cast<SE2State>(current_state);
    std::shared_ptr<SE2State> goal_se2state = std::static_pointer_cast<SE2State>(goal_state_);
    return (*current_se2state) == (*goal_se2state);
  }

  float getHeuristicValue(std::shared_ptr<State> current_state)
  {
    std::shared_ptr<SE2State> current_se2state = std::dynamic_pointer_cast<SE2State>(current_state);
    std::shared_ptr<SE2State> goal_se2state = std::dynamic_pointer_cast<SE2State>(goal_state_);
    return manhattanDistanceHeuristic(*current_se2state, *goal_se2state);
  }
  std::vector< std::pair<std::shared_ptr<State> , float > > getSuccessors(std::shared_ptr<State> current_state)
  {
    
    std::shared_ptr<SE2State> current_se2state = std::dynamic_pointer_cast<SE2State>(current_state);
    std::vector<std::pair<std::shared_ptr<State>, float> > successors;

    SE2State moves[8] = {
      SE2State(1, 0, 0.0),
      SE2State(-1, 0, 0.0),
      SE2State(0, 1, 0.0),
      SE2State(0, -1, 0.0),
      SE2State(1, 1, 0.0),
      SE2State(-1, -1, 0.0),
      SE2State(-1, 1, 0.0),
      SE2State(1, -1, 0.0)
    };
    for(int i = 0; i < 8; i++)
    {
      int next_x = current_se2state->getX() + moves[i].getX();
      int next_y = current_se2state->getY() + moves[i].getY();
      int next_theta = current_se2state->getYaw() + moves[i].getYaw();
      if(isFree(next_x, next_y))
      {
        float cost = sqrt((float)(moves[i].getX()*moves[i].getX() + moves[i].getY()*moves[i].getY()));
        pair<std::shared_ptr<State>, float> successor_pair(std::static_pointer_cast<State>(std::make_shared<SE2State>(next_x, next_y, next_theta)), cost);
        successors.push_back(successor_pair);
      }
    }
    return successors;
  }
};

unsigned int map_width, map_height;
unsigned char* map;

    bool isFree(int x, int y)
    {
      for(int i = -5; i < 5; i++)
      {
        for(int j = -5; j < 5; j++)
        {
          if( x + i < 0 && x + i >= map_width ||  y + j < 0 || y+j >= map_height || map[(y+j)*map_width + x + i] > 10)
            return false;
        }
      }
    return true;
    }

bool requestSearch(pathfinding_msgs::PathRequest::Request &req,
                   pathfinding_msgs::PathRequest::Response &res)
{

  std::shared_ptr<SE2State> start_state = std::make_shared<SE2State>((int)req.startState.x, (int)req.startState.y, (float)req.startState.theta);
  std::shared_ptr<SE2State> goal_state = std::make_shared<SE2State>((int)req.goalState.x, (int)req.goalState.y, (float)req.goalState.theta);
  SearchProblem problem = SearchProblem(start_state, goal_state);

  AStar astar(&problem);
  try
  {
    double beginning = ros::Time::now().toSec();
    vector<std::shared_ptr<State>> path = astar.search();
    ROS_INFO("A* found a path in %f", ros::Time::now().toSec() - beginning);
    ROS_INFO("Path length = %ld", path.size());
    for(int i = 0; i < path.size(); i++)
    {
      path[i]->print();
      std::shared_ptr<SE2State> se2state = std::dynamic_pointer_cast<SE2State>(path[i]);
      pathfinding_msgs::State state;
      state.x = se2state->getX();
      state.y = se2state->getY();
      state.theta = se2state->getYaw();
      res.path.state.push_back(state);
    }
  }
  catch(const char* e)
  {
    std::string error(e);
    ROS_ERROR("%s", error.c_str());
    res.error = error;
  }
  return true;
}

void mapReceiver(const nav_msgs::OccupancyGrid mapData)
{
    ROS_INFO("Map dimensions: %d %d\n", (unsigned int)mapData.info.width, (unsigned int)mapData.info.height);

    map_width = mapData.info.width;
    map_height = mapData.info.height;
    map = (unsigned char*) malloc(map_width*map_height);
//    memcpy(map, mapData.data, map_width*map_height);
    
//    std::ofstream file;
//    file.open("/tmp/ros/mapa.txt");
    for(int i=0; i < mapData.info.height; i++)
    {
        for(int j=0; j < mapData.info.width; j++)
        {
            unsigned char cell = (unsigned char) mapData.data[mapData.info.width*i+j];
            map[map_width*i+j] = cell;
//            if (cell < 10)
//                file << ".";
//            else if (cell > 200)
//                file << "#";
//            else
//                file << "*";
        }
//        file << "\n";
    }
//    file.close();
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astarSearch");


    ros::NodeHandle n;

    ros::Subscriber mapSub = n.subscribe("map", 10, astar_search::mapReceiver);
    ros::ServiceServer service = n.advertiseService("astarSearch", astar_search::requestSearch);
    ROS_INFO("A* search startup initiated");
    ros::spin();

    return 0;
}
