#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class SubscribeAndRequest
{
public:
  SubscribeAndRequest()
  {

    // Define a client service capable of requesting services from command_robot
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10, &SubscribeAndRequest::process_image_callback, this);

  }

  void process_image_callback(const sensor_msgs::Image img)
  {
    int white_pixel = 255;
    int location = 0;
    int nr_location = 0;
    bool is_ball = false;
    

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height * img.step; i++) {
        if ((img.data[i] == white_pixel) && (img.data[i++] == white_pixel) && (img.data[i++] == white_pixel)) {
            // x coordinate can be calculated with the formula:
            location += (i % img.step) / 3;
            nr_location++;
            is_ball = true;
            is_changed_ = true;
            //break;
        }
    }
    //ROS_INFO_STREAM("is ball?: " + std::to_string(is_ball));

    if (!is_ball && is_changed_){
        drive_robot(0, 0);
        is_changed_ = false;
        return;
    }
    else if (is_ball) {
        
        location /= nr_location;
        ROS_INFO_STREAM("Ball location: " + std::to_string(location));
        //ROS_INFO_STREAM("location: " + std::to_string(location) + " , img.height: " + std::to_string(img.height) + " , img.width: " + std::to_string(img.width));
        if (location < img.width/3){
            //turn left
            drive_robot(0.0, 1);
        }
        else if (location > 2*img.width/3){
            //turn right
            drive_robot(0.0, -1);
        }
        else {
            // forward
            drive_robot(0.4, 0);
        }
        
    }
    else {
        return;
    }

    

  }

  // This function calls the command_robot service to drive the robot in the specified direction
  void drive_robot(float lin_x, float ang_z)
  {
    // TODO: Request a service and pass the velocities to it to drive the robot
    if (lin_x == 0){
        ROS_INFO_STREAM("Cannot see the ball, stopping");
    }
    else {
        ROS_INFO_STREAM("Chasing the white ball");
    }

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client_.call(srv))
        ROS_ERROR("Failed to call service drive to target");

  }

private:
  ros::NodeHandle n_; 
  ros::Subscriber sub_;
  // Define a global client that can request services
  ros::ServiceClient client_;
  bool is_changed_ = true;

};//End of class SubscribeAndRequest

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "process_image");

  //Create an object of class SubscribeAndRequest that will take care of everything
  SubscribeAndRequest SARObject;

  ros::spin();

  return 0;
}