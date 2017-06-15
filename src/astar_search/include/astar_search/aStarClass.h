#include <vector>
#include <queue>
#include <map>
#include <cstdlib>
#include <cstring>
#include <algorithm>

#include "ros/ros.h"

using std::vector;
using std::priority_queue;
using std::make_pair;

struct State
{
private:
    int x,y;
    float theta;
    
public:
    State();
    State(int, int, float);
    int getX() const;
    int getY() const;
    float getTheta() const;

};

inline bool operator==(const State& lhs, const State& rhs)
{
    return lhs.getX() == rhs.getX()
        && lhs.getY() == rhs.getY()
        && lhs.getTheta() == rhs.getTheta();
}

inline bool operator<(const State& lhs, const State& rhs)
{
    if(lhs.getX() != rhs.getX())      return lhs.getX() < rhs.getX();
    else if(lhs.getY() != rhs.getY()) return lhs.getY() < rhs.getY();
    else                              return lhs.getTheta() < rhs.getTheta();
}


class AStar
{            
private:

    typedef int (*HeurFnType)(State, State);

    static const int DEFAULT_COST = 1;

    struct Map
    {
        unsigned char *map;
        int height, width;
        
        bool isFree(int x, int y)
        {
            for(int i = -5; i < 5; i++)
            {
                for(int j = -5; j < 5; j++) 
                {
                    if( x + i < 0 && x + i >= width ||  y + j < 0 || y+j >= height || map[(y+j)*width + x + i] > 10)
                        return false;
                }
            }
            return true;
//            return x >= 0 && x < width && y >= 0 && y < height
//                   && map[y*width + x] < 10;
        }
    } map;
    State start_state;
    State goal_state;
    vector<State> path;

    HeurFnType heurFn;    

    volatile bool map_received;
    volatile bool searching;
    volatile bool pathFound;

    State getStartState();
    bool isGoalState(State);
    vector<State> getSuccessors(State);

public:
    AStar(State, State);
    ~AStar();

    vector<State> getPath();

    void setMap(int, int, unsigned char*);
    void setHeuristic(HeurFnType);

    vector<State> search();
};
