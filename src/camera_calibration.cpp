#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <camera_calibration/calibrate.h>
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"


namespace enc = sensor_msgs::image_encodings;

bool cal_b = false;	// Only calibrate and save when service is called


// Common class for performing calibration, given inpute images
class calibration
{
public:
	int count;	// Count for reading images
	float square_size;
	std::string file_name;
	bool is_done;
	int no_of_images;
	cv::Mat image;
	int flag;

	std::vector<cv::Point2f> centers; //this will be filled by the detected centers
	std::vector<std::vector<cv::Point3f> > object_points;
	std::vector<std::vector<cv::Point2f> > image_points;

	cv::Mat intrinsic; // FC1 for specifying no of channels
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;	


	calibration(int n, std::string f_name)
	{
		// initialization
		count = 0;  //count to stop after 20 images
		square_size = 1.1; // in cm
		is_done = false;
		no_of_images = n;
		file_name = f_name;
	}

	bool calibrate_camera_intrinsic(cv::Mat im)
	{
		image = im;
		if (is_done && (count == no_of_images))
			return true;
		// put code
		cv::Size patternsize(4,11); // number of centers
		/*
		cv::SimpleBlobDetector::Params params;
		params.maxArea = 10e6;
		cv::Ptr<cv::SimpleBlobDetector> blobDetector = cv::SimpleBlobDetector::create(params);
		*/

		/*
		bool patternfound = cv::findCirclesGrid(image, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);

	    // Subpixel refinement
	    cv::Mat src_gray;
	    cv::cvtColor(image, src_gray, cv::COLOR_BGR2GRAY);
	    cv::Size winSize = cv::Size( 5, 5 );
	    cv::Size zeroZone = cv::Size( -1, -1 );
	    cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );
	    cornerSubPix( src_gray, centers, winSize, zeroZone, criteria );	//Subpixel refinement function
	    */

	    bool patternfound = image_pyramid(4); // takes depth of pyramid as argument, image size reduced by 2^(depth-1)

		// Draw corners on image
		cv::drawChessboardCorners(image, patternsize, cv::Mat(centers), patternfound);


		// Calculate 3D world points corresponding to centers
		std::vector<cv::Point3f> obj;
		for (int i = 0; i < 11; i++)
	      for (int j = 0; j < 4; j++)
	        obj.push_back(cv::Point3f((float)(2*j + i % 2)* square_size, (float)i * square_size, 0.0));

		count++;
		if (patternfound)
		{
			image_points.push_back(centers);
			object_points.push_back(obj);
			ROS_INFO("%d", count);
		}


		if (count == no_of_images)
		{
			calibrate_now();
			is_done = true;
		}

		display_image(4);
		return is_done;
	}

	void calibrate_now()
	{	
		intrinsic = cv::Mat(3, 3, CV_32FC1); // FC1 for specifying no of channels
		distCoeffs = cv::Mat::zeros(8, 1, CV_32F);
		//flag = 0;
		//flag |= CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6; // CV_CALIB_FIX_K1 | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 |
		calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
		
		// Save in yaml file
		save_data();

		// Display intrinsic K matrix
		std::cout << intrinsic << "\n";
	}

	// Builds pyramid and finds corners
	bool image_pyramid(int depth)
	{
		/*
		std::vector<cv::Mat> pyramid;
		cv::Mat dest;
		cv::cvtColor(image, dest, cv::COLOR_BGR2GRAY);
		pyramid.push_back(dest)
		// build Pyramid
		for (int i = 0; i < depth-1; i++)
		{
			cv::pyrDown(pyramid[i], dest, cv::Size(pyramid[i].cols/2, pyramid[i].rows/2));
			pyramid[i+1].push_back(dest);
		}

		bool found = false;
		int i = 3;
		cv::Size patternsize(4,11); // number of centers
		while (!found && i >= 0)
		{
			found = cv::findCirclesGrid(pyramid[i], patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);
			i--;
		}
		if (!found)
			return false;
		// Subpixel refinement
	    cv::Mat src_gray;
	    cv::cvtColor(image, src_gray, cv::COLOR_BGR2GRAY);
	    cv::Size winSize = cv::Size( 5, 5 );
	    cv::Size zeroZone = cv::Size( -1, -1 );
	    cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 40, 0.001 );
		for (int j = i; j>0; j--)
		{
			cv::pyrUp(pyramid[i], dest, cv::Size(pyramid[i].cols*2, pyramid[i].rows*2));
			pyramid[i-1] = dest;
			cornerSubPix( pyramid[i-1], centers, winSize, zeroZone, criteria );	//Subpixel refinement function
		}*/

		return true;
	}

	void save_data(){
		//already have this function
		cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
		fs << "K" << intrinsic;
		fs << "D" << distCoeffs;
	}

	void display_image(int scale_down){
		resize(image, image, cv::Size(image.cols/scale_down, image.rows/scale_down));
		cv::imshow("found corners", image);
		cv::waitKey(100);
	}

	void reset()
	{
		// Reset all variables to initial values
		count = 0;
		is_done = false;

		object_points.clear();
		image_points.clear();
	}

};

// Initialize class objects
calibration a(20, "/home/mihird/mihir_ws/src/camera_calibration/config/intrinsic_matrix");
calibration b(20, "/home/mihird/mihir_ws/src/camera_calibration/config/intrinsic_matrix1");


// Service callback function
bool calibrateCallback(camera_calibration::calibrate::Request &req, camera_calibration::calibrate::Response &res)
{
	// Listen and calibrate
	if (req.a == 1)
	{
		ROS_INFO("Calibrating");
		res.success = "Success";
		cal_b = true;

	}
	else
		res.success = "Fail";
	return true;
}


void save_data(std::string file_name, cv::Mat intrinsic, cv::Mat distCoeffs){
	cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
	fs << "K" << intrinsic;
	fs << "D" << distCoeffs;
}

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

	a.calibrate_camera_intrinsic(cv_ptr->image);
	if (cal_b){
		b.calibrate_camera_intrinsic(cv_ptr->image);
		if(b.is_done){
			// Reset object
			cal_b = false;
			b.reset();
		}
	}
	

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "camera_calibration_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback);
	ros::ServiceServer service = nh.advertiseService("calibrate", calibrateCallback);
	ros::spin();

} 
