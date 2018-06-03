#include <vector>

#include <queue>
#include <unordered_map>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <memory>

namespace state_search{ 

struct State
{
public:
  virtual std::size_t hash();
  virtual inline bool operator==(const std::shared_ptr<State> other);
};

class ProblemDefinition
{
public:
  virtual std::shared_ptr<State> getStartState() = 0;
  virtual bool isGoalState(std::shared_ptr<State>) = 0; 
  //TODO find a way to split getSuccessorStates function header
  virtual std::vector<
    std::pair<std::shared_ptr<State>,
    float>
  > getSuccessorStates(std::shared_ptr<State>) = 0;
  virtual float getHeuristicValue(std::shared_ptr<State>) = 0;

protected:
  std::shared_ptr<State> start_state_, goal_state_;
};

namespace astar_class{

using state_search::State;
using state_search::ProblemDefinition;

class AStar
{            
public:
  AStar(state_search::ProblemDefinition*);

  int getOpenedNodesCount();

  bool isPathFound();

  std::vector<std::shared_ptr<State> > getPath();
  std::vector<std::shared_ptr<State> > search();

private:
  std::vector<std::shared_ptr<State> > path_;
  ProblemDefinition* problem_;

  bool search_finished_;
  bool path_found_;
  int opened_nodes_counter_;
};

} } //end namespace state_search::astar_class
