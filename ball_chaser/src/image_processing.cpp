#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "ball_chaser/NormalizedPosition.h"

#include <string>


class ImageProcessor
{
   	ros::ServiceClient client
    void ProcessImageCallback(const sensor_msgs::Image& img);

public:
    ImageProcessor() {}
    // Run the image processor
    void run();

};

void ImageProcessor::chase(float linear_x, float angular_z)
	
	// Request velocities applied to differential drive
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x;
    srv.request.angular_z = angular_z;

    // Call /ball_chaser/command_robot service
    if (not drive_srv_client_.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

void ImageProcessor::ProcessImageCallback(const sensor_msgs::ImagePtr& img)
{
   
    int white_pixel = 255;
	float x_position;
	
    for (int column = 0; column < img.height; ++column)
	{
        for (int row = 0; row < img.step; ++row)
		{
            int pixel = column * img.step + row;
            if (img.data[pixel] == white_pixel)
			{
                x_position = static_cast<float>(row);
                if (x_position < img.step / 3) {
                    chase(0.5, 0.5);
                } else if (x_position > img.step / 1.5) {
                    chase(0.5, -0.5);
                } else {
                    chase(0.5, 0.);
                }
                return;
            }
        }
    }

    chase(0., 0.);
    return;
}
	
void ImageProcessor::Run()
{
    ros::NodeHandle n;
	
	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");


    // Subscribe to /camera/rgb/image_raw topic to read the image data
    ros::Subscriber camera_subscriber = n.subscribe("/camera/rgb/image_raw", 10,
                                                &ImageProcessor::process_image_callback);

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