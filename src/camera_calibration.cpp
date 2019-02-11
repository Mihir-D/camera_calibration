#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>


namespace enc = sensor_msgs::image_encodings;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	//convert rosmsg to Mat object
	cv_bridge::CvImageConstPtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}
	//Display image
	cv::imshow("Display window", cv_ptr->image);                   // Show our image inside it.
    cv::waitKey(100);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_calibration_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
	ros::spin();

} 
