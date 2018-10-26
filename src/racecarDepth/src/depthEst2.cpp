//"depthEst2.cpp"
//1.Detect objects; 2.Publish global coordinates of the centre poing in bounding box
//@author Guanghui Ma
//last modified: 27/9/2018
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <racecarDepth/ObsPose.h> //Define a information
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/visualization/pcl_visualizer.h> //Used for visualization

// Explanation：
//Output from "ObsPose"：
/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32[] x
float32[] y
float32[] dist
float32[] angle
//----Output------
(x,y): global coordinates of object
dist: relative distance
angle
***Considering more than one targets, the output is a vector with 4 values
*/
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;
//
const cv::Mat Matrix = (cv::Mat_<double>(3, 3) << 501.9406, 0.0, 330.2514, 0.0, 502.2422, 234.5579, 0.0, 0.0, 1.0);
const cv::Mat coefficients = (cv::Mat_<double>(5, 1) << 0.0266, -0.0613, 0.0, 0.0, -0.0015);//Parameters of camera
cv::Point curpoint;
bool bObsOK = 0;
ros::Publisher obsdepth_pub;
// Declare functions
void rgb_callback(const sensor_msgs::ImageConstPtr &msg );
void points_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
void modelstatus_callback(const gazebo_msgs::ModelStatesConstPtr &msg);
double colorfilter(cv::Mat picture);//To extract blue colour
geometry_msgs::Pose egopose;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_estimation");
    curpoint.x = -1;
    curpoint.y = -1;
    egopose.position.x = 0;
    egopose.position.y = 0;
    egopose.position.z = 0;
    egopose.orientation.w = 1;
    egopose.orientation.x = 0;
    egopose.orientation.y = 0;
    egopose.orientation.z = 0;
    ros::NodeHandle nh;
    cv::namedWindow("RGB_image");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/kinect/rgb/image_raw", 1, rgb_callback);
    ros::Subscriber sub2 = nh.subscribe("/kinect/depth/points",1,points_callback);
    ros::Subscriber sub3 = nh.subscribe("/gazebo/model_states",1,modelstatus_callback);
    obsdepth_pub = nh.advertise<racecarDepth::ObsPose>("obspose",3);
    ros::spin();
    return 0;
}

void modelstatus_callback(const gazebo_msgs::ModelStatesConstPtr &msg)
{
    int i;
    for(i = 0; i < msg->name.size(); i++)
    {
        string strtmp = msg->name[i];
        if(strtmp == "racecar")
            break;
    }
    egopose = msg->pose[i]; // Pose of our car
}
void rgb_callback(const sensor_msgs::ImageConstPtr &msg )
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat frame = cv_ptr->image.clone();
    if (frame.empty())
    {
        ROS_ERROR("Empty image");
        return;
    }
    cv::Mat pic = frame.clone();
    //To find centre point
    colorfilter(frame);
    cvtColor(frame, frame, CV_BGR2GRAY);//Grey image
    threshold(frame, frame, 10, 255, CV_THRESH_BINARY);//Binarization, didnot use otsu here.
    vector<vector<cv::Point>> contours;
    findContours(frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//Contour
    //drawContours(frame, contours, -1, 255, 2, 8);
    if(contours.size() <= 0 || contours[0].size() < 5)
    {
        cv::imshow("RGB_image",pic);//Does not work properly
        return ;
    }
    cv::Rect box;
    box = boundingRect(contours[0]);//Bounding box
    
    curpoint.x = box.x + box.width / 2;
    curpoint.y = box.y + box.height / 2;//Return centre point and regard it as target poing.
    cv::circle(pic, curpoint, 2, cv::Scalar(90, 243, 39), -1);
    cv::imshow("RGB_image",pic);
    bObsOK = 1;
}

void points_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    //Receive points cloud
    PointCloud::Ptr cloudrcv(new PointCloud);
    pcl::fromROSMsg(*msg, *cloudrcv);
    PointT pttmp;//Define a temp point
    int ptwidth = cloudrcv->width;
    //To check if the target has been found
    if(bObsOK && curpoint.x > 0 && curpoint.y > 0)
    {
        bObsOK = 0;
        pttmp = cloudrcv->points[curpoint.y * ptwidth + curpoint.x]; //Return the coordinates of centre point.
        racecarDepth::ObsPose pub_msg;
        pub_msg.header = msg->header;
        // Transfer to global coordinate
        tf::Quaternion q1;
        tf::quaternionMsgToTF(egopose.orientation, q1);
        tf::Matrix3x3 R =  tf::Matrix3x3(q1);
        tf::Point obspos(pttmp.z, -pttmp.x, pttmp.y); //Kinect's coordinates is differet from ROS
        //cout<<"local obspos: "<<obspos[0] <<' ' <<obspos[1]<<' ' <<obspos[2]<<endl;
        tf::Point egopos(egopose.position.x, egopose.position.y, egopose.position.z);
        obspos = R.transpose() * obspos + egopos;
        
        pub_msg.x.push_back(obspos[0]);
        pub_msg.y.push_back(-obspos[1]); //I made a mistake at first:(, modified on 27/9/2018
        float dist = sqrt(pttmp.x * pttmp.x + pttmp.z * pttmp.z);
        pub_msg.dist.push_back(dist);
        float tmpangle = atan2(pttmp.x,pttmp.z);
        pub_msg.angle.push_back(tmpangle);

        obsdepth_pub.publish(pub_msg); //Publish pub_msg
        // cout<<"ego pose: "<<egopose.position.x <<' ' <<egopose.position.y<<endl;
        cout<< obspos[0] << ' '<< -obspos[1] << ' ' << dist<<' ' << tmpangle<<endl;
    }
}

double colorfilter(cv::Mat picture)//Colour filter
{
	cv::Mat hsv;
	cvtColor(picture, hsv, CV_BGR2HSV);
	for (int i = 0; i < picture.rows; i++)
	{
		for (int j = 0; j<picture.cols; j++)
		{
			double H = hsv.at<cv::Vec3b>(i, j)[0];
			double S = hsv.at<cv::Vec3b>(i, j)[1];
			double V = hsv.at<cv::Vec3b>(i, j)[2];

			if ((H >= 100 && H <= 124) && S>43 && V>46)// From google
			{
				//picture.at<uchar>(i, j) = 0;
				picture.at<cv::Vec3b>(i, j)[0] = picture.at<cv::Vec3b>(i, j)[0];
				picture.at<cv::Vec3b>(i, j)[1] = picture.at<cv::Vec3b>(i, j)[1];
				picture.at<cv::Vec3b>(i, j)[2] = picture.at<cv::Vec3b>(i, j)[2];
			}
			else
			{
				//picture.at<uchar>(i, j) = 255;
				picture.at<cv::Vec3b>(i, j)[0] = 0;
				picture.at<cv::Vec3b>(i, j)[1] = 0;
				picture.at<cv::Vec3b>(i, j)[2] = 0;
			}


		}
	}
	//imshow("1", picture);
	return 0;
}
