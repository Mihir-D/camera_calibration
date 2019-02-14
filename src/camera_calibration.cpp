#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <iostream>
#include <camera_calibration/calibrate.h>
#include <opencv2/features2d.hpp>


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
		count = 0;
		square_size = 1.0;
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
		bool patternfound = cv::findCirclesGrid(image, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);

		// Draw corners on image
		cv::drawChessboardCorners(image, patternsize, cv::Mat(centers), patternfound);


		std::vector<cv::Point3f> obj;
		for (int i = 0; i < 4; i++)
	      for (int j = 0; j < 11; j++)
	        obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0.0));

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
		
		//display_image();
		return is_done;
	}

	void calibrate_now()
	{	
		intrinsic = cv::Mat(3, 3, CV_32FC1); // FC1 for specifying no of channels
		calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
		
		// Save in yaml file
		save_data();

		// Display intrinsic K matrix
		std::cout << intrinsic << "\n";
	}

	void save_data(){
		//already have this function
		cv::FileStorage fs(file_name, cv::FileStorage::WRITE);
		fs << "K" << intrinsic;
		fs << "D" << distCoeffs;
	}

	void display_image(){
		resize(image, image, cv::Size(image.cols/4, image.rows/4));
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
	/*
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

	// *************** Detect circles *********************
    cv::Size patternsize(4,11); // number of centers
	//cv::Mat gray = ....; // source image
	std::vector<cv::Point2f> centers; //this will be filled by the detected centers

	bool patternfound = cv::findCirclesGrid(cv_ptr->image, patternsize, centers, cv::CALIB_CB_ASYMMETRIC_GRID);

	// Draw corners on image
	cv::drawChessboardCorners(cv_ptr->image, patternsize, cv::Mat(centers), patternfound);
	// ****************************************************

	// *************** calibrate camera *********************
	float square_size = 1.0;

	// Change objects
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
		cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1); // FC1 for specifying no of channels
	    cv::Mat distCoeffs;
	    std::vector<cv::Mat> rvecs;
	    std::vector<cv::Mat> tvecs;	
		calibrateCamera(object_points, image_points, cv_ptr->image.size(), intrinsic, distCoeffs, rvecs, tvecs);
		
		// Save in yaml file
		std::string file_name = "/home/mihird/mihir_ws/src/camera_calibration/config/intrinsic_matrix";
		save_data(file_name, intrinsic, distCoeffs);

		// Display intrinsic K matrix
		std::cout << intrinsic << "\n";
		
	} */
	// ***************************************************

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
