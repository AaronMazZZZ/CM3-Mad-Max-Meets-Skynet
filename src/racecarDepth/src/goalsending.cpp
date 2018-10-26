//"goalsending.cpp"
//Send goal to top layer
//@author: Xuze Wang, Guanghui Ma
//last modified: 2/10/2018

#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <racecar_depth/ObsPose.h> //
#include <geometry_msgs/PoseStamped.h>
#include <racecarDepth/ObsPose.h> //Define output
#include <tf/transform_datatypes.h>

using namespace std;

const float fDistThr = 1.0; //Assume that attacking success ratio is 100, i.e. once we find the target we can assume the attack is finished.
int nObsOk = 0;            //Check if the object has been detected.
float fObsposex;
float fObsposey;

void obs_callback(const racecarDepth::ObsPoseConstPtr &msg);
int main(int argc, char **argv)
{
    ros::init(argc, argv, "goalset");
    ros::NodeHandle n;
    float fgoalx;
    float fgoaly;
    float fgoalyaw; 
//Check distance 
    if (argc <= 1)
    {
        //Using the default Goal:
        ROS_INFO("Using the default settings");
        fgoalx = 15.8;
        fgoaly = 26.6;
        fgoalyaw = 0;
    }
    else if (argc == 3)
    {
        // Set the goal:
        fgoalx = atof(argv[1]);
        fgoaly = atof(argv[2]);
        fgoalyaw = 0;
    }
    else if(argc == 4)
    {
        fgoalx = atof(argv[1]);
        fgoaly = atof(argv[2]);
        fgoalyaw = atof(argv[3]);
    }
    else
    {
        ROS_ERROR("Goal setting error");
        return 0;
    }
    ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);//This can be found in rqt_graph
    ros::Subscriber obs_sub = n.subscribe("obspose", 1, obs_callback);
    ros::Rate loop_rate(1);
    int count = 1;
    while (ros::ok())
    {
        geometry_msgs::PoseStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.header.seq = count++;
        
        //send the goal points.
        if (nObsOk == 0)
        {
            //If there is no target, send out default goal
            msg.pose.position.x = fgoalx;
            msg.pose.position.y = fgoaly;
            msg.pose.position.z = 0.2;
        }
        else
        {
            //If the competitor car is found, send its coordinates
            msg.pose.position.x = fObsposex;
            msg.pose.position.y = fObsposey;
            msg.pose.position.z = 0.2;
            if(++nObsOk>3) //To avoid fluctration from slow testing
            {
                nObsOk = 0;
            }
        }
        //Set pose0 0 0
        tf::Quaternion q;
        q.setRPY(0, 0, fgoalyaw/180*M_PI);
        tf::quaternionTFToMsg(q, msg.pose.orientation);
        //Publish msg
        goal_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void obs_callback(const racecarDepth::ObsPoseConstPtr &msg)
{
    // Subscribe msg
    if(msg->dist[0] > fDistThr)
    {
        fObsposex = msg->x[0];
        fObsposey = msg->y[0];
        nObsOk = 1;
    }
    else{
        nObsOk = 0;
    }
}
