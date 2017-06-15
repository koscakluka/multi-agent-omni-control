//Filename astarSearch

//#include "headers/astarSearch.h"
#include "ros/ros.h"
#include "astar_search/aStarClass.h"

#include <fstream>
#include <string>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "std_msgs/String.h"

#include "pathfinding_msgs/State.h"
#include "pathfinding_msgs/Path.h"
#include "pathfinding_msgs/PathRequest.h"

namespace astar_search
{
unsigned int map_width, map_height;
unsigned char* map;

int manhattanDistanceHeuristic(State current_state, State goal_state)
{
    return abs(goal_state.getX() - current_state.getX()) + abs(goal_state.getY() - current_state.getY());
}

bool requestSearch(pathfinding_msgs::PathRequest::Request &req,
                   pathfinding_msgs::PathRequest::Response &res)
{
    State start_state = State((int)req.startState.x, (int)req.startState.y, (float)req.startState.theta);
    State goal_state = State((int)req.goalState.x, (int)req.goalState.y, (float)req.goalState.theta);
    ROS_INFO("(%d, %d, %f) -> (%d, %d, %f)",
              (int)req.startState.x, (int)req.startState.y, (float)req.startState.theta,
              (int)req.goalState.x, (int)req.goalState.y, (float)req.goalState.theta);

    AStar astar(start_state, goal_state);
    astar.setMap(map_height, map_width, map);
    astar.setHeuristic(manhattanDistanceHeuristic);

    try
    {
        double beginning = ros::Time::now().toSec();
        vector<State> path = astar.search();
        ROS_INFO("A* found a path in %f", ros::Time::now().toSec() - beginning);
        for(int i = 0; i < path.size(); i++)
        {
            pathfinding_msgs::State state;
            state.x = path[i].getX();
            state.y = path[i].getY();
            state.theta = path[i].getTheta();
            res.path.state.push_back(state);
        }
        return true;
    }
    catch(const char* e)
    {
        std::string error(e);
        ROS_ERROR("%s", error.c_str());
        res.error = error;
        return true; 
    }
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

    ros::ServiceServer service = n.advertiseService("astarSearch", astar_search::requestSearch);
    ros::Subscriber mapSub = n.subscribe("map", 10, astar_search::mapReceiver);
    ROS_INFO("A* search startup initiated");
    ros::spin();

    return 0;
}
