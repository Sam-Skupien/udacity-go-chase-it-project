#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/Float64.h>
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities

bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res) {
	
	ROS_INFO("Received request to go to linear_x: %1.2f angular_z: %1.2f", (float)req.linear_x,
	(float)req.angular_z);

	// Create a motor_command object of type geometry_msgs::Twist
        geometry_msgs::Twist motor_command;

        // Set wheel velocities, forward [0.5, 0.0]
        motor_command.linear.x = req.linear_x;
        motor_command.angular.z = req.angular_z;

        // Publish angles to drive the robot
        motor_command_publisher.publish(motor_command);

	res.msg_feedback = "Received command to go to linear_x: " + std::to_string(req.linear_x) + " angular_z: "
            		   + std::to_string(req.angular_z) + ".\n";

	ROS_INFO_STREAM(res.msg_feedback);

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle n;

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::ServiceServer server = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
    
    ros::spin();

    return 0;
}