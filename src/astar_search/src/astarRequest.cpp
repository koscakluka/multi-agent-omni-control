#include "ros/ros.h"

#include "pathfinding_msgs/State.h"
#include "pathfinding_msgs/Path.h"
#include "pathfinding_msgs/PathRequest.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

int main(int argc, char **argv){

    ros::init(argc, argv, "astarRequest");

    if(argc != 7)
    {
        ROS_ERROR("astarRequest usage: startStateX startStateY startStateTheta goalStateX goalStateY goalStateTheta");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<pathfinding_msgs::PathRequest>("astarSearch");
    ros::Publisher path_publisher = n.advertise<nav_msgs::Path>("path", 1000);

    pathfinding_msgs::PathRequest pathReq;

    //Load parameters in request
    pathReq.request.startState.x = atoi(argv[1]);
    pathReq.request.startState.y = atoi(argv[2]);
    pathReq.request.startState.theta = atof(argv[3]);

    pathReq.request.goalState.x = atoi(argv[4]);
    pathReq.request.goalState.y =  atoi(argv[5]); 
    pathReq.request.goalState.theta =  atof(argv[6]);

    if(client.call(pathReq))
    {
        if(pathReq.response.error.size() == 0)
        {
            nav_msgs::Path nav_path;
            nav_path.header.frame_id = "odom";
            nav_path.header.stamp = ros::Time::now();
            ROS_INFO("Path");
            for(int i = 0; i < pathReq.response.path.state.size(); i++)
            {
                pathfinding_msgs::State state = pathReq.response.path.state[i];
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.pose.position.x = (float) state.x * 0.05;
                pose_stamped.pose.position.y = (float) state.y * 0.05;
                nav_path.poses.push_back(pose_stamped);
                ROS_INFO("(%d, %d, %f)", state.x, state.y, state.theta);
                
            }
            ros::Rate ros_loop(100);
            for(int i = 0; i < 40; i++)
            {
                path_publisher.publish(nav_path);
                ros_loop.sleep();
                ros::spinOnce();
            }
        }
        else
        {
            ROS_ERROR("%s", pathReq.response.error.c_str());
        }
    }
    else
    {
        ROS_ERROR("Received no response from the search server.");
        return 1;
    }

    return 0;
}
