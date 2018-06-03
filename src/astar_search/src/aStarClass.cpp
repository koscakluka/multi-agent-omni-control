#include "astar_search/aStarClass.h"

namespace state_search{

/*! \class State
 * An abstract class used to define a State used by AStar class.
 *
 * This class assumes nothing about the state it only requires certain methods
 * needed by the AStar class to find a path.
 */

/*!
 * A hash function of the state.
 * Required by AStar class so state can be added into an unordered map.
 */
std::size_t State::hash()
{
    return 0;
}

/*!
 * A equality operator of the state
 * Required by AStar class so state can be added into an unordered map.
 */
inline bool State::operator==(const std::shared_ptr<State> other)
{
    return true;
}

/*! \class ProblemDefinition
 * An abstract class used to define a problem used by AStar class.
 *
 * This class assumes nothing about the problem it only requires certain
 * methods needed by the AStar class to find a path. Problem is defined by it's
 * start and goal state as well as it's method for getting successor states
 * from a ceratain state.
 *
 * \var ProblemDefinition::start_state_
 * Pointer to the state from which the search is started.
 *
 * \var ProblemDefinition::goal_state_
 * Pointer to the state where the search ends.
 *
 * \fn virtual std::shared_ptr<State> ProblemDefinition::getStartState()
 * Getter for the start state.
 * 
 * Required by AStar class so it can get the start state via a function without
 * the ability to influence it and also forcing the implementation to use the abstract
 * State class.
 *
 * \return Pointer to the start state.
 *
 * \fn bool ProblemDefinition::isGoalState(std::shared_ptr<State> state)
 * Checks whether a given pointer to a State is a goal state.
 *
 * Required by AStar class so it can terminate the search when it reaches the
 * goal state without the ability to influence it and forcing the implementation
 * to use the abstract State class.
 * \param state Pointer to the state being checked.
 *
 * \return True if the given pointer to a State is a goal state.
 */

namespace astar_class
{

using state_search::State;
using state_search::ProblemDefinition;

/*!
 * A structure containing a node used in AStar search.
 * One node describes a certain state because only knowing the state isn't
 * enough to describe the path that lead to it or it's value.
 */
struct Node
{
private:
  std::shared_ptr<State> state_;
  std::shared_ptr<Node> parent_;
  float cost_;
  float heuristic_;

public:
  /*!
   * Constructs a Node using provided state description.
   *
   * @param state     Shared pointer to the described state.
   * @param parent    Shared pointer to the node the state was reached from.
   * @param cost      Total cost required to reach the state.
   * @param heuristic Heuristic value of the state.
   */
  Node(std::shared_ptr<State> state, std::shared_ptr<Node> parent, float cost, float heuristic):
    state_(state), parent_(parent), cost_(cost), heuristic_(heuristic){};

  std::shared_ptr<State> getState() const {return state_;}
  std::shared_ptr<Node> getParent() const {return parent_;}
  float getCost() const {return cost_;}
  float getHeuristic() const {return heuristic_;}
};

/*!
 * A node comparator class used to compare two Nodes inside a priority queue.
 * Comparison is done based on the sum of Node cost and Node's heuristic value.
 */
class NodeComparator
{
public:
  bool operator()(const std::shared_ptr<Node> lhs,
                  const std::shared_ptr<Node> rhs) const
  {
    return lhs->getCost() + lhs->getHeuristic() > rhs->getCost() + rhs->getHeuristic();
  }
};

/*!
 * A state hash class used to calculate a State's hash for an unordered map.
 * It calls the states own hash function.
 */
struct StateHash
{
public:
    std::size_t operator()(const std::shared_ptr<State>& state) const
    {
        return state->hash();
    }
};

/*!
 * A state comparator class used to compare two states inside an unordered map.
 * It compares states for equality using the equality operator.
 */
struct StateEqual
{
public:
  bool operator()(const std::shared_ptr<State>& lhs,
                  const std::shared_ptr<State>& rhs) const
  {
    return *lhs == rhs;
  }
};


/*!
 * Reconstructs a path from the received Node.
 * @param node Shared pointer to the last node in the path.
 *
 * @return An ordered vector of shared pointers to the states in the path.
 */
std::vector<std::shared_ptr<State> > reconstructPath(std::shared_ptr<Node> node)
{
  if(node == nullptr) return std::vector<std::shared_ptr<State> >();
  auto path = reconstructPath(node->getParent());
  path.push_back(node->getState());
  return path;
}



/*! \class AStar
 *
 *
 */

/*!
 * Contructs an AStar object using a supplied problem definition.
 * 
 * @param problem A pointer to the problem definition used in the search
 */
AStar::AStar(ProblemDefinition* problem):
  problem_(problem), search_finished_(false), path_found_(false),
  opened_nodes_counter_(0) {};



/*!
 * Gets number of nodes opened while searching for the path.
 *
 * \return Number of nodes opened while searching for the path.
 */
int AStar::getOpenedNodesCount()
{
  return opened_nodes_counter_;  
}



/*!
 * Checks if path was found during the last search.
 *
 * \return True if path was found or if search is
 */
bool AStar::isPathFound()
{
  return search_finished_ && path_found_;
}



/*!
 * Gets the path if one exists.
 * 
 * \return An vector of shared pointers to the states in the path sorted from
 *         from the start state to the goal state if path was found or an empty
 *         vector otherwise.
 */
std::vector<std::shared_ptr<State> > AStar::getPath()
{
  if(isPathFound())
  {
    return path_;
  }
  else
  {
    return std::vector<std::shared_ptr<State> >();
  }
}

/*!
 * Runs the A* search.
 * Restarts opened states counter if the same search is run multiple times.
 *
 * \return An vector of shared pointers to the states in the path sorted from
 *         from the start state to the goal state.
 */
std::vector< std::shared_ptr<State> > AStar::search()
{

  std::priority_queue<
    std::shared_ptr<Node>,
    std::vector<std::shared_ptr<Node> >,
    NodeComparator> opened_nodes;

  std::unordered_map<
    std::shared_ptr<State>,
    float,
    StateHash,
    StateEqual> known_states;

  std::shared_ptr<Node> reached_goal_node = nullptr;
  bool search_succesful = false;

  //set class variables to their initial state
  opened_nodes_counter_ = 0;
  search_finished_ = false;
  path_found_ = false;

  //add start state as first node
  opened_nodes.push(std::make_shared<Node>(
    problem_->getStartState(),
    nullptr,
    0,
    problem_->getHeuristicValue(problem_->getStartState()))
  );
  
  while(!opened_nodes.empty())
  {
    auto current_node = opened_nodes.top();
    opened_nodes.pop();

    auto remembered_state = known_states.find(current_node->getState());
    bool was_state_visited = remembered_state != known_states.end();
    if(was_state_visited && current_node->getCost() >= remembered_state->second)
    {
      continue;
    }
  
    opened_nodes_counter_++;

    if(problem_->isGoalState(current_node->getState()))
    {
      search_succesful = true;
      reached_goal_node = current_node;
      break;
    }

    //remember current state's cost
    known_states.insert(std::make_pair<std::shared_ptr<State>, float>(
      current_node->getState(),
      current_node->getCost()
    ));

    auto next_states = problem_->getSuccessorStates(current_node->getState());
    for(int i = 0; i < next_states.size(); i++)
    {
      auto next_state = next_states[i].first;
      float action_cost = next_states[i].second;

      auto remembered_state = known_states.find(next_state);
      bool was_state_visited = remembered_state != known_states.end();
      int  new_node_cost = current_node->getCost() + action_cost;
      if(!was_state_visited || new_node_cost < remembered_state->second)
      {
        //add new state as new node
        opened_nodes.push(std::make_shared<Node>(
          next_state,
          current_node,
          new_node_cost,
          problem_->getHeuristicValue(next_state)
        ));
      }
    }
  }
  search_finished_ = true;

  if(search_succesful)
  {
    path_ = reconstructPath(reached_goal_node);
    path_found_ = true;
  }

  return path_;
}

} } //end namespace state_search::astar_class
