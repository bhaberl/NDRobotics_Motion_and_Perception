#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "ball_chaser/DriveToTarget.h"

#include <string>


class ImageProcessor
{
    ros::ServiceClient client_;
    

public:
    ImageProcessor() {}
    // Run the image processor
    void run();
    void process_image_callback(const sensor_msgs::Image& img);
    void drive_robot(float linear_x, float angular_z);

};

void ImageProcessor::drive_robot(float linear_x, float angular_z)
{
    // Request velocities 
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;
    if(!client_.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");
   
}

void ImageProcessor::process_image_callback(const sensor_msgs::Image& img)
{
    int white_pixel = 255;
    float white_pixel_x;
    float linear_x = 0.0;
    float angular_z = 0.0;
   	
    for (int row = 0; row < img.height; row++)
    {
        for (int column = 0; column < (img.step-3); column+=3)
		{
		int pixel = row * img.step + column;
		if ((img.data[pixel] == white_pixel) && (img.data[pixel+1] == white_pixel) && (img.data[pixel+2] == white_pixel))
	    {
			white_pixel_x = static_cast<float>(column);
			ROS_INFO_STREAM("FOUND WHITE PIXEL");
			ROS_INFO_STREAM(white_pixel_x);
			if (white_pixel_x < img.step / 3)
			{
				linear_x = 0.0;
				angular_z = -0.5;
			}
			else if (white_pixel_x > img.step / 1.5)
			{
                linear_x = 0.0;
				angular_z = 0.5;
			}
			else
			{
                linear_x = 0.2;
				angular_z = 0.0;
            }
			drive_robot(linear_x, angular_z);
			return;
            }
        }
    }    
    drive_robot(linear_x, angular_z);
    return;
}
	
void ImageProcessor::run()
{
    ros::NodeHandle n;
	
    // Define a client service capable of requesting services from command_robot
    client_ = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");


    // Subscribe to /camera/rgb/image_raw topic to read the image data
    ros::Subscriber camera_subscriber = n.subscribe("/camera/rgb/image_raw", 10, &ImageProcessor::process_image_callback, this);

    // Handle ROS communication events
    ros::spin();
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "process_image");

    // Initialize image processor
    ImageProcessor processor;

    // Run
    processor.run();

    return 0;
}
