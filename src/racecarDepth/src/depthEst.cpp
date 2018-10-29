//version1 depthEst.cpp
//This one is just a test file. It does not work well. Please use 'depthEst2.cpp'.
//@author Guanghui Ma
#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "racecarDepth/ObsPose.h" 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// PCL
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/visualization/pcl_visualizer.h> 


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace std;

const cv::Mat Matrix = (cv::Mat_<double>(3, 3) << 501.9406, 0.0, 330.2514, 0.0, 502.2422, 234.5579, 0.0, 0.0, 1.0);
const cv::Mat coefficients = (cv::Mat_<double>(5, 1) << 0.0266, -0.0613, 0.0, 0.0, -0.0015);
cv::Point curpoint;
bool bObsOK = 0;
ros::Publisher obsdepth_pub;

void rgb_callback(const sensor_msgs::ImageConstPtr &msg );
void points_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
double colorfilter(cv::Mat picture);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_estimation");
    curpoint.x = -1;
    curpoint.y = -1;
    ros::NodeHandle nh;
    cv::namedWindow("RGB_image");
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/kinect/rgb/image_raw", 1, rgb_callback);
    ros::Subscriber sub2 = nh.subscribe("/kinect/depth/points",1,points_callback);
    obsdepth_pub = nh.advertise<racecarDepth::ObsPose>("obspose",3);
    ros::spin();
    return 0;
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

    colorfilter(frame);
    cvtColor(frame, frame, CV_BGR2GRAY);
    threshold(frame, frame, 10, 255, CV_THRESH_BINARY);
    vector<vector<cv::Point>> contours;
    findContours(frame, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //drawContours(frame, contours, -1, 255, 2, 8);
    if(contours.size() <= 0 || contours[0].size() < 5)
    {
        cv::imshow("RGB_image",pic);
        return ;
    }
    cv::Rect box;
    box = boundingRect(contours[0]);
    
    curpoint.x = box.x + box.width / 2;
    curpoint.y = box.y + box.height / 2;
    cv::circle(pic, curpoint, 2, cv::Scalar(90, 243, 39), -1);
    cv::imshow("RGB_image",pic);
    bObsOK = 1;
}

void points_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{

    PointCloud::Ptr cloudrcv(new PointCloud);
    pcl::fromROSMsg(*msg, *cloudrcv);
    PointT pttmp;
    int ptwidth = cloudrcv->width;

    if(bObsOK && curpoint.x > 0 && curpoint.y > 0)
    {
        bObsOK = 0;

        pttmp = cloudrcv->points[curpoint.y * ptwidth + curpoint.x]; 
        racecarDepth::ObsPose pub_msg;
        pub_msg.header = msg->header;
        pub_msg.x.push_back(pttmp.x);
        pub_msg.y.push_back(pttmp.z); 
        float dist = sqrt(pttmp.x * pttmp.x + pttmp.z * pttmp.z);
        pub_msg.dist.push_back(dist);
        float tmpangle = atan2(pttmp.x,pttmp.z);
        pub_msg.angle.push_back(tmpangle);
        obsdepth_pub.publish(pub_msg);
        cout<< pttmp.x << ' '<< pttmp.z << ' ' << dist<<' ' << tmpangle<<endl;
    }
}

double colorfilter(cv::Mat picture)
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

			if ((H >= 100 && H <= 124) && S>43 && V>46)
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
