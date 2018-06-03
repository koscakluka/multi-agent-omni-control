#include <algorithm>
#include "ros/ros.h"

#include "search_msgs/PathRequest.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <ompl/base/SpaceInformation.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace state_search{ namespace rrt_search{

int map_width, map_height;
float map_resolution;
unsigned char* map;
const int MAP_STATE_FREE_THRESHOLD = 10;
bool map_received = false;
float biggestX, lowestX, biggestY, lowestY;

bool isStateValid(const ob::State *state)
{
  const ob::SE2StateSpace::StateType& pos = *state->as<ob::SE2StateSpace::StateType>();
  int mapX = floor(pos.getX()/map_resolution);
  int mapY = floor(pos.getY()/map_resolution);
  for (int i = -7; i <= 7; i++)
  {
    for (int j = -4; j <= 4; j++)
    {
      bool is_x_out_of_bounds = mapX + i < 0 || mapX + i >= map_width;
      bool is_y_out_of_bounds = mapY + j < 0 || mapY + j >= map_height;
      if(is_x_out_of_bounds || is_y_out_of_bounds)
        return false;

      char current_cell = map[(mapY + j) * map_width + mapX + i];
      bool is_state_free = current_cell <= MAP_STATE_FREE_THRESHOLD;
      if(!is_state_free)
        return false;
    }
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

    biggestX = 0;
    lowestX = map_width;
    biggestY = 0;
    lowestY = map_height;

    for(int i = 0; i < mapData.info.height; i++)
    {
        for(int j = 0; j < mapData.info.width; j++)
        {
            unsigned char cell = (unsigned char) mapData.data[mapData.info.width*i +j];
            map[map_width*i + j] = cell;

            if (cell <= MAP_STATE_FREE_THRESHOLD)
            {
                biggestX = std::max<float>(biggestX, j);
                lowestX = std::min<float>(lowestX, j);
                biggestY = std::max<float>(biggestY, i);
                lowestY = std::min<float>(lowestY, i);
            }
        }
    }
    biggestX *= map_resolution;
    lowestX *= map_resolution;
    biggestY *= map_resolution;
    lowestY *= map_resolution;

    map_received = true;
}

og::PathGeometric plan(float startX, float startY, float goalX, float goalY)
{
    ob::StateSpacePtr space(new ob::SE2StateSpace);

    //Set the bounds to match 
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, lowestX);
    bounds.setHigh(0, biggestX);
    bounds.setLow(1, lowestY);
    bounds.setHigh(1, biggestY);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    og::SimpleSetup ss(space);

    // Set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = startX;
    start[1] = startY;
    start[2] = 0;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = goalX;
    goal[1] = goalY;
    goal[2] = 0;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);


    // Create and set a RRT planner
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    planner->as<og::RRT>()->setRange(map_resolution);
    planner->as<og::RRT>()->setGoalBias(0.1);
    ss.setPlanner(planner);

    ob::PlannerStatus solved = ss.solve(5.00);
    if (solved)
    {
        ROS_INFO("Path found");
        og::PathGeometric path = ss.getSolutionPath();
        ROS_INFO("Path lenght: %ld", path.getStateCount());
        return path;
    }
    else
    {
        ROS_INFO("Not able to find a path");
        return og::PathGeometric(ss.getSpaceInformation());
    }
}

bool requestSearch(search_msgs::PathRequest::Request &req,
                   search_msgs::PathRequest::Response &res)
{
  if(!state_search::rrt_search::map_received)
  {
    res.error = "Map hasn't been loaded.";
    res.is_error = true;
    return true;
  }

  float startX = req.start_state.position.x;
  float startY = req.start_state.position.y;

  float goalX = req.goal_state.position.x;
  float goalY = req.goal_state.position.y;

  og::PathGeometric path = state_search::rrt_search::plan(startX, startY, goalX, goalY);
  if(path.getStateCount() > 0)
  {
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = "/odom";
    nav_path.header.stamp = ros::Time::now();
    for(int i = 0; i < path.getStateCount(); i++)
    {
      geometry_msgs::PoseStamped pose;
      const ob::SE2StateSpace::StateType& pos = *(path.getState(i))->as<ob::SE2StateSpace::StateType>();
      float x = pos.getX();
      float y = pos.getY();
      float theta = pos.getYaw();
      pose.pose.position.x = x;
      pose.pose.position.y = y;
    
      nav_path.poses.push_back(pose);
    }
    res.is_error = false;
    return true;
  }
  else
  {
    res.error = "Path could not be found.";
    res.is_error = true;
    return true;
  }
}

} } //end namespace state_search::rrt_search


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_search");

    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    ros::ServiceServer pathPub = n.advertiseService("rrt_search", state_search::rrt_search::requestSearch);
    ros::Subscriber mapSub = n.subscribe("map", 10, state_search::rrt_search::mapReceiver);

    ros::spin();

}

