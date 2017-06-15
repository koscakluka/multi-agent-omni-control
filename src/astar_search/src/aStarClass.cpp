#include "astar_search/aStarClass.h"

State::State(){}

State::State(int x, int y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

int State::getX() const
{
    return x;
}

int State::getY() const
{
    return y;
}

float State::getTheta() const
{
    return theta;
}

namespace astar_search
{

struct Node
{
private:
    State state;
    Node *parent;
    int cost;
    int heuristic;

public:
    Node(State state, Node *parent, int cost, int heuristic)
    {
    this->state = state;
    this->parent = parent;
    this->cost = cost;
    this->heuristic = heuristic;
    }

    //getters
    State getState() const {return state;}
    Node* getParent() const {return parent;}
    int getCost() const {return cost;}
    int getHeuristic() const {return heuristic;}
};

class NodeComparator
{
public:
    bool operator()(Node *lhs, Node *rhs)
    {
        return lhs->getCost() + lhs->getHeuristic() > rhs->getCost() + rhs->getHeuristic();
    }
};

int defaultHeurFn(State, State){return 0;}
}

AStar::AStar(State start_state, State goal_state)
{
    this->start_state = start_state;
    this->goal_state = goal_state;
    this->heurFn = astar_search::defaultHeurFn;
    map_received = false;
    searching = false;
    map.map = NULL;
}

AStar::~AStar()
{
    if(map.map != NULL){
        free(map.map);
    }
}

State AStar::getStartState()
{
    return start_state;
}

bool AStar::isGoalState(State state)
{
    return state == goal_state;
}

vector<State> AStar::getSuccessors(State state)
{
    vector<State> successors;

    State moves[4] = {State(1, 0, 0.0), State(-1, 0, 0.0), State(0, 1, 0.0), State(0, -1, 0.0)};
    for(int i = 0; i < 4; i++)
    {
        int next_x = state.getX() + moves[i].getX();
        int next_y = state.getY() + moves[i].getY();
        int next_theta = state.getTheta() + moves[i].getTheta();
        if(!map.isFree(next_x, next_y)) continue;
        successors.push_back(State(next_x, next_y, next_theta));
    }
    return successors;

}

void AStar::setMap(int height, int width, unsigned char* map)
{
    this->map.height = height;
    this->map.width = width;
    this->map.map = (unsigned char*) malloc(height*width);
    memcpy(this->map.map, map, height*width);
    map_received = true;
}

void AStar::setHeuristic(HeurFnType heurFn)
{
    this->heurFn = heurFn;
}

vector<State> AStar::search()
{
    if(!map.isFree(start_state.getX(), start_state.getY()) || !map.isFree(goal_state.getX(), goal_state.getY()))
    {
        ROS_ERROR("Cannot calculate path due to unreachable start and goal states!");
        throw "Cannot calculate path due to unreachable start and goal states!";
    }
    priority_queue<astar_search::Node*, vector<astar_search::Node*>, astar_search::NodeComparator> opened_nodes;
    vector<astar_search::Node*> created_nodes;
    std::map<State, int> known_states;
    bool success = false;
    astar_search::Node* goal_node;
    int counter = 0;

    opened_nodes.push(new astar_search::Node(getStartState(), NULL, 0, heurFn(getStartState(), goal_state))); //TODO fix memory leak
    known_states.insert(make_pair<State, int>(getStartState(), 0));

    while(!opened_nodes.empty())
    {
        astar_search::Node* current_node = opened_nodes.top();
        opened_nodes.pop();

        //ROS_INFO("(%d %d)", current_node->getState().getX(), current_node->getState().getY());
        //ros::Duration(3).sleep();
        
        std::map<State, int>::iterator node_cost_old_it = known_states.find(current_node->getState());

        if(node_cost_old_it != known_states.end() && current_node->getCost() > node_cost_old_it->second)
        {
            continue;
        }

        if(isGoalState(current_node->getState()))
        {
            success = true;
            goal_node = current_node;
            break;
        }

        known_states.insert(make_pair<State, int>(current_node->getState(), current_node->getCost()));

        counter++;
        if(!(counter % 10000))
        {
            ROS_INFO("%d", counter);
        }
        vector<State> next_states = getSuccessors(current_node->getState());
        for(int i = 0; i < next_states.size(); i++)
        {
            State next_state = next_states[i];
            //ROS_INFO("potential (%d %d)", next_state.getX(), next_state.getY());
            node_cost_old_it = known_states.find(next_state);
//            if(node_cost_old_it != known_states.end() && current_node->getCost() + DEFAULT_COST > node_cost_old_it->second)
            if(node_cost_old_it != known_states.end())
            {
                //ROS_INFO("passed");
                continue;
            }
            
            astar_search::Node* new_node = new astar_search::Node(next_state, current_node, current_node->getCost() + DEFAULT_COST, heurFn(next_state, goal_state));
            opened_nodes.push(new_node);
            //ROS_INFO("cost %d", new_node->getCost());
            known_states.insert(make_pair<State, int>(next_state, current_node->getCost() + DEFAULT_COST));
            created_nodes.push_back(new_node); //remember that the node was created
        }
    }
    
    if(success)
    {
        astar_search::Node *current_node = goal_node;
        while(current_node != NULL)
        {
            path.push_back(current_node->getState());
            current_node = current_node->getParent();
        }
        std::reverse(path.begin(), path.end());
    }

    //delete all created nodes
    for(int i = 0; i < created_nodes.size(); i++)
    {
        delete created_nodes[i];
    }
    
    return path;
}
