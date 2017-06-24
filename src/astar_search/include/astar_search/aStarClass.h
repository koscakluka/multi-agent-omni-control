#include <vector>
#include <queue>
#include <unordered_map>
#include <map>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <memory>

#include "ros/ros.h"

using std::vector;
using std::priority_queue;
using std::make_pair;
using std::pair;

struct State
{
  virtual ~State(){};
  virtual void print() const {ROS_INFO("Empty state");};
  virtual std::size_t hash(){return 0;};
  virtual inline bool operator==(const std::shared_ptr<State> other) = 0;
  virtual inline bool operator<(const std::shared_ptr<State> other) = 0;
};

class ProblemDefinition
{
protected:
  std::shared_ptr<State> start_state_, goal_state_;

public:
  virtual std::shared_ptr<State> getStartState() = 0;
  virtual bool isGoalState(std::shared_ptr<State>) = 0; 
  virtual vector< std::pair<std::shared_ptr<State>, float> > getSuccessors(std::shared_ptr<State>) = 0 ;
  virtual float getHeuristicValue(std::shared_ptr<State>) = 0;
};


class AStar
{            
private:
  vector< std::shared_ptr<State> > path;
  ProblemDefinition* problem_;

  volatile bool searching;
  volatile bool pathFound;

public:
  AStar(ProblemDefinition*);
  AStar(State&, State&);

  vector< std::shared_ptr<State> > getPath();
  vector< std::shared_ptr<State> > search();
};
