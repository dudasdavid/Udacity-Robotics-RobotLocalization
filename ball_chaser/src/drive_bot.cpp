#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//TODO: Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

class ServiceAndPublish
{
public:
  ServiceAndPublish()
  {
    //Topic you want to publish
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to subscribe
    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    service_ = n_.advertiseService("/ball_chaser/command_robot", &ServiceAndPublish::handle_drive_request, this);

    ROS_INFO("Ready to send drive requests!");

  }

  // This callback function executes whenever a drive_request service is requested
  // void handle_drive_request(const SUBSCRIBED_MESSAGE_TYPE& input)
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
  {

    ROS_INFO("DriveToTarget Request received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher_.publish(motor_command);

    // Return a response message
    res.msg_feedback = "cmd_vel command set - linear_x: " + std::to_string(motor_command.linear.x) + " , angular_z: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }


private:
  // Create a ROS NodeHandle object
  ros::NodeHandle n_;
  ros::Publisher motor_command_publisher_;
  ros::ServiceServer service_;

};//End of class ServiceAndPublish

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    //Create an object of class ServiceAndPublish that will take care of everything
    ServiceAndPublish SAPObject;

    ros::spin();

    return 0;
}