#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

#include <string>


class BotManager
{
    ros::Publisher motor_command_publisher_;

    // Handles drive request whenever the service "/ball_chaser/command_robot" is called
    bool PerformDriveRequest(ball_chaser::DriveToTarget::Request& req,
                             ball_chaser::DriveToTarget::Response& res);

public:
    BotManager() {}
    
    void run();


};

void BotManager::run()
{
// Create a ROS NodeHandle object
    ros::NodeHandle n;
	
	// Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
	// Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot",
                                                        &BotManager::PerformDriveRequest,
                                                        this);
  
   
    ros::spin();
}

bool BotManager::PerformDriveRequest(ball_chaser::DriveToTarget::Request& req,
                                    ball_chaser::DriveToTarget::Response& res)
{
    //ROS_INFO("DriveToTargetRequest received: linear_x: %1.2f, angular_z: %1.2f", req.linear_x, req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher_.publish(motor_command);

    // Return a response message
    res.msg_feedback = "linear.x: " + std::to_string(req.linear_x) + ", angular.z: " + std::to_string(req.angular_z);

    ROS_INFO_STREAM(res.msg_feedback);
    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Initialize bot driver
    BotManager bot;

    // Run driver
    bot.run();

    return 0;
}
