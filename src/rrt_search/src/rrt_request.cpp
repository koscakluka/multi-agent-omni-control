#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "search_msgs/PathRequest.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "rrt_request");

  if(argc != 5)
  {
    ROS_ERROR("rrt_request usage: startStateX startStateY goalStateX goalStateY");
    return 1;
  }

  ros::NodeHandle n;

  ros::ServiceClient rrt_search_client = n.serviceClient<search_msgs::PathRequest>("rrt_search");
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 100);

  search_msgs::PathRequest path_req;

  path_req.request.start_state.position.x = atof(argv[1]);
  path_req.request.start_state.position.y = atof(argv[2]);

  path_req.request.goal_state.position.x = atof(argv[3]);
  path_req.request.goal_state.position.y = atof(argv[4]);

  if(rrt_search_client.call(path_req))
  {
    if(!path_req.response.is_error)
    {
      ros::Rate ros_loop(100);
      //send path 40 times at frequency of 100Hz
      //required for Rviz to display it
      for(int i = 0; i < 40; i++)
      {
        path_pub.publish(path_req.response.path);
        ros::spinOnce();
        ros_loop.sleep();
      }
    }
    else
    {
      //ROS_ERROR("%s", path_req.response.error);
      return 1;
    }
  }
  else
  {
      ROS_ERROR("Received no response from the search server.");
  }

  return 0;
}
