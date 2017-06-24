#include "astar_search/aStarClass.h"

namespace astar_search
{

struct Node
{
private:
  std::shared_ptr<State> state_;
  std::shared_ptr<Node> parent_;
  float cost_;
  float heuristic_;

public:
  Node(std::shared_ptr<State> state, std::shared_ptr<Node> parent, float cost, float heuristic):
    state_(state), parent_(parent), cost_(cost), heuristic_(heuristic){};

  //getters
  std::shared_ptr<State> getState() const {return state_;}
  std::shared_ptr<Node> getParent() const {return parent_;}
  float getCost() const {return cost_;}
  float getHeuristic() const {return heuristic_;}
};

class NodeComparator
{
public:
  bool operator()(const std::shared_ptr<Node> lhs, const std::shared_ptr<Node> rhs) const
  {
    return lhs->getCost() + lhs->getHeuristic() > rhs->getCost() + rhs->getHeuristic();
  }
};

float defaultHeuristicFunction(State&, State&){return 0;}
}

struct StateHash
{
public:
    std::size_t operator()(const std::shared_ptr<State>& state) const
    {
        return state->hash();
    }
};

struct StateEqual
{
public:
  bool operator()(const std::shared_ptr<State>& lhs, const std::shared_ptr<State>& rhs) const
  {
    return *lhs == rhs;
  }
};

AStar::AStar(ProblemDefinition* problem) : problem_(problem), searching(false){};

vector< std::shared_ptr<State> > AStar::search()
{
//  //TODO check start/goal state for availability

//  if(!map.isFree(start_state.getX(), start_state.getY()) || !map.isFree(goal_state.getX(), goal_state.getY()))
//  {
//      ROS_ERROR("Cannot calculate path due to unreachable start and goal states!");
//      throw "Cannot calculate path due to unreachable start and goal states!";
//  }
  priority_queue<
    std::shared_ptr<astar_search::Node>,
    std::vector<std::shared_ptr<astar_search::Node> >,
    astar_search::NodeComparator
  > opened_nodes;
  std::unordered_map<std::shared_ptr<State>, float, StateHash, StateEqual> known_states;
  bool success = false;
  std::shared_ptr<astar_search::Node> goal_node;
  int counter = 0;

  opened_nodes.push(std::make_shared<astar_search::Node>(problem_->getStartState(), nullptr, 0, problem_->getHeuristicValue(problem_->getStartState())));

  while(!opened_nodes.empty())
  {
    std::shared_ptr<astar_search::Node> current_node = opened_nodes.top();
    opened_nodes.pop();

    std::unordered_map<std::shared_ptr<State>, float, StateHash, StateEqual>::iterator node_cost_old_it = known_states.find(current_node->getState());

    if(node_cost_old_it != known_states.end() && current_node->getCost() >= node_cost_old_it->second)
    {
      continue;
    }

    if(problem_->isGoalState(current_node->getState()))
    {
      success = true;
      goal_node = current_node;
      break;
    }

    known_states.insert(make_pair<std::shared_ptr<State>, float>(current_node->getState(), current_node->getCost()));

    counter++;
    if(!(counter % 1000))
    {
      ROS_INFO("%d", counter);
    }
    vector< std::pair<std::shared_ptr<State>, float> > next_states = problem_->getSuccessors(current_node->getState());
    for(int i = 0; i < next_states.size(); i++)
    {
      std::shared_ptr<State> next_state = next_states[i].first;
      float cost = next_states[i].second;
      node_cost_old_it = known_states.find(next_state);
      if(node_cost_old_it != known_states.end() && current_node->getCost() + cost >= node_cost_old_it->second)
//      if(node_cost_old_it != known_states.end())
      {
        continue;
      }
            
      opened_nodes.push(std::make_shared<astar_search::Node>(next_state, current_node, current_node->getCost() + cost, problem_->getHeuristicValue(next_state)));
    }
  }
    
  if(success)
  {
    std::shared_ptr<astar_search::Node> current_node = goal_node;
    while(current_node != nullptr)
    {
      path.push_back(current_node->getState());
      current_node = current_node->getParent();
    }
    std::reverse(path.begin(), path.end());
  }

  return path;
}
