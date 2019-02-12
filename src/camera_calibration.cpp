#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>


namespace enc = sensor_msgs::image_encodings;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	// Convert rosmsg to Mat object
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

    cv::Size patternsize(4,11); // number of centers
	//cv::Mat gray = ....; // source image
	std::vector<cv::Point2f> centers; //this will be filled by the detected centers

	bool patternfound = cv::findCirclesGrid(cv_ptr->image, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);

	cv::drawChessboardCorners(cv_ptr->image, patternsize, cv::Mat(centers), patternfound);

	// Display image
	cv::imshow("Display window", cv_ptr->image);
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
