#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher_node");
	ros::NodeHandle nh;

	std::string pkg_path;
    if(nh.getParam("/image_publisher_node/path", pkg_path))
    	ROS_INFO("TRUE");
    else{
    	ROS_INFO("FALSE");
    	return 0;
    }

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/image_raw", 1);


	std::string path = pkg_path + "/images/*.JPG"; // /home/mihird/mihir_ws/src/camera_calibration
	std::vector<cv::String> fn;
	std::vector<cv::Mat> images;
	cv::glob(path,fn,true); // recurse

	for (size_t k=0; k<fn.size(); ++k)
	{
	     cv::Mat im = cv::imread(fn[k], CV_LOAD_IMAGE_COLOR);
	     //cv::waitKey(30);
	     if (im.empty()) //only proceed if sucsessful
	     {
	     	ROS_INFO("Could not open or find the image\n") ;
	     	continue;
	     }
	     
	     images.push_back(im);

	}

    //cv::imshow( "Display window", images[10] );                   // Show our image inside it.
    //cv::waitKey(0);

	//sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[3]).toImageMsg();

	// Create messages for each image
	std::vector<sensor_msgs::ImagePtr> msgs;
	for (int i = 0; i <images.size(); i++)
	{
		msgs.push_back(cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[i]).toImageMsg());
	}

	ros::Rate loop_rate(5);
	int i = 0;	
	while(nh.ok()) 
	{
	    pub.publish(msgs[i]);	//publish each image on ros topic
	    ros::spinOnce();
	    loop_rate.sleep();
	    i++;
	    if (i==msgs.size()) 
	    	i = 0;
	}

}
