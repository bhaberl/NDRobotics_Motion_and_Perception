#include <cmath>
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/duration.h"
#include "ball_chaser/DriveToTarget.h"
#include "ball_chaser/NormalizedPosition.h"
#include "ball_chaser/robot_status.hpp"


class BallChaser
{
    ros::ServiceClient drive_srv_client_;
    RobotStatus robot_status_;
    bool is_ball_found_;
    bool are_surroudings_scanned_;

    // Callback for the subscription to /ball_chaser/ball_hor_loc.
    // If ball was found in the image, it commands the drive_bot
    // (through service /ball_chaser/command_robot) to drive towards
    // the ball
    void LocateBallCallback(const ball_chaser::NormalizedPosition& pos);

    void LookForWhiteBall(double& linear_x, double& angular_z);

public:
    BallChaser() :
        is_ball_found_{false},
        are_surroudings_scanned_{false}
    {}
    // Run the ball chaser
    void Run();
};

void BallChaser::Run()
{
    ros::NodeHandle nodeh;

    // Specify service and type of message of the client
    drive_srv_client_ = nodeh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /ball_chaser/ball_norm_position topic to read the
    // normalized position of the ball as seen from the camera
    ros::Subscriber sub_hloc = nodeh.subscribe("/ball_chaser/ball_norm_position", 10,
                                                &BallChaser::LocateBallCallback,
                                                this);

    // Handle ROS communication events
    ros::spin();
}

void BallChaser::LocateBallCallback(const ball_chaser::NormalizedPosition& pos)
{
    double linear_x_vel{1.0};
    double angular_z_vel{0.0};
    const double max_angular_vel{0.5};
    const double proximity_threshold_reduce{-0.6};
    const double proximity_threshold_stop{-0.8};
    const double turning_threshold = 0.2;
    const double turning_threshold_sq = turning_threshold * turning_threshold;
    if(pos.contains_object)
    {
        if (not is_ball_found_)
        {
            ROS_INFO_STREAM("Camera sees white ball");
            is_ball_found_ = true;
        }
        are_surroudings_scanned_ = false;
        robot_status_.SetToChasing();

        const double hor_pos = pos.horizontal;
        const double ver_pos = pos.vertical;
        double sign = 1.0;
        if (hor_pos < 0.0)
            sign = -1.0;



        // We want angular_z to be a function of hor_pos that goes
        // smoothly from 0 to max_angular_vel in the interval
        // s[0, 1], and such that most of its change happens around
        // turning_threshold.
        // We also would like angular_z to have derivatives
        // in the extremes close to 0, so that:
        //   a) It glues smoothly to the constant function described
        //      in the case hor_pos > turning_threshold
        //   b) For values of hor_pos close enough to 0, there is not
        //      much variation of angular_z, so that there is no rapid
        //      oscillarions and we are as close as driving in straight
        //      line as possible
        //   c) We can mirror for [-turning_threshold, 0]
        // Function f(x) = (1 + tanh(4*x))/2 behaves this exact way
        // in [-2, 2] (with horizontal asymptote angular_z = 1), so all we need to do is:
        //   1) Multiply f by max_angular_vel
        //   2) substitute x by the right
        //      transformation g(hor_pos) that sends [0 turning_threshold]
        //      to [-2, 0]
        // For example, for turning_threshold = 0.3 and max_angular_vel = 0.5,
        // we can see the graph of the absolute value of angular_z here:
        // https://www.wolframalpha.com/input/?i=0.5*(1+%2B+tanh(4*(2*abs(x)%2F0.3+-+2)))%2F2+from+-1+to+1
        double x = 2.0 * fabs(hor_pos) / turning_threshold - 2.0;
        angular_z_vel = sign * max_angular_vel * (1.0 + std::tanh(2 * x)) / 2.0;

        // If the white pixel found is too close to the bottom of the image,
        // we don't move forward anymore
        if (ver_pos < proximity_threshold_stop)
        {
            linear_x_vel = 0.0;
        }
        else
        {
            // We take a parabole with respect to hor_pos so that
            // less we need to turn, the faster we can move and
            // constant to 0.5 when the horizonal normalized position
            // is beyond the turning_threshold.
            if(fabs(hor_pos) > turning_threshold)
                linear_x_vel = 0.5;
            else
                linear_x_vel = 1.0 - 0.5 * hor_pos * hor_pos / turning_threshold_sq;

            // We reduce in the case we are getting close to the ball
            if (ver_pos < proximity_threshold_reduce)
            {
                linear_x_vel *= (proximity_threshold_stop - ver_pos)
                             /  (proximity_threshold_stop - proximity_threshold_reduce);
            }
        }

    }
    else
    {
        if (robot_status_.IsChasing())
            robot_status_.SetToStop();
        LookForWhiteBall(linear_x_vel, angular_z_vel);
    }

    // Request velocities applied to differential drive
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = linear_x_vel;
    srv.request.angular_z = angular_z_vel;

    // Call /ball_chaser/command_robot service
    if (not drive_srv_client_.call(srv))
        ROS_ERROR("Failed to call service /ball_chaser/command_robot");

}

void BallChaser::LookForWhiteBall(double& linear_x, double& angular_z)
{
    const double rotation_std_angular_z_velocity = 0.85;

    // If the robot is static and hasn't been static for more than
    // 2 seconds, it stays static. If it was stopped for longer,
    // we start scanning
    if(robot_status_.IsStopped())
    {
        if (robot_status_.TimeStoppedInSeconds() < 2.0)
        {
            linear_x = 0.0;
            angular_z = 0.0;
            robot_status_.SetToStop();
            return;
        }
        else
        {
            robot_status_.SetToScanning();
        }
    }

    if(not are_surroudings_scanned_)
    {
        linear_x = 0.0;
        angular_z = rotation_std_angular_z_velocity;

        double angle_rotated = robot_status_.TimeScanningInSeconds() * rotation_std_angular_z_velocity;

        if(angle_rotated > 2 * M_PI)
        {
            are_surroudings_scanned_ = true;

        }
    }
    else
    {
        linear_x = 0.0;
        angular_z = 0.0;
        robot_status_.SetToStop();
    }
}


int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "chase_ball");

    // Initialize ball chaser
    BallChaser chaser;

    // Run chaser
    chaser.Run();

    return 0;
}