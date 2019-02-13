#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>


namespace enc = sensor_msgs::image_encodings;


int count = 0;

/*
void intrinsic_calibration(std::vector<std::vector<cv::Point3f>>& object_points, std::vector<std::vector<cv::Vec2f>>& image_points)
{
	cv::Mat intrinsic = Mat(3, 3, CV_32FC1);
    cv::Mat distCoeffs;
    std::vector<Mat> rvecs;
    std::vector<Mat> tvecs;	
	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
}
*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	count++;
	
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

	float square_size = 1.0;

	std::vector<cv::Point3f> obj;
	for (int i = 0; i < 4; i++)
      for (int j = 0; j < 11; j++)
        obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0.0));

	std::vector<std::vector<cv::Point3f> > object_points;
	std::vector<std::vector<cv::Point2f> > image_points;
	if (patternfound)
	{
		image_points.push_back(centers);
		object_points.push_back(obj);
	}

	if (count == 20)
	{
		//intrinsic_calibration(object_points, image_points);
		cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
	    cv::Mat distCoeffs;
	    std::vector<cv::Mat> rvecs;
	    std::vector<cv::Mat> tvecs;	
		calibrateCamera(object_points, image_points, cv_ptr->image.size(), intrinsic, distCoeffs, rvecs, tvecs);
		// Display the matrix
		std::cout << intrinsic;
		
		}
	}

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_calibration_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
	ros::spin();

} 
