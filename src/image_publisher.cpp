#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_publisher_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise("/image_raw", 1);
	int i = 1;
	std::string path = "/home/mihird/mihir_ws/src/camera_calibration/images/";
	cv::Mat image = cv::imread(path + "1.jpeg", CV_LOAD_IMAGE_COLOR);
	cv::waitKey(30);
	if(! image.data )                              // Check for invalid input
    {
        ROS_INFO("Could not open or find the image\n") ;
        return -1;
    }

    cv::imshow( "Display window", image );                   // Show our image inside it.

    cv::waitKey(0);

	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

	ros::Rate loop_rate(5);	
	while(nh.ok()) {
	    pub.publish(msg);
	    ros::spinOnce();
	    loop_rate.sleep();
	}

}
