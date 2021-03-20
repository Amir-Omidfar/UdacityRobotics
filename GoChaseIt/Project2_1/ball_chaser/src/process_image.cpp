#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Chasing the ball");
    
    // Request service with velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the DriveToTarget service and pass the requested velocities
    if (!client.call(srv)) {
	    ROS_ERROR("Failed to call the service DriveToTarget.");
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
	float lin_x = 0;
  	float ang_z = 0;
    int white_pixel = 255;
  	unsigned int rows = img.height;
	unsigned int cols = img.width;
  	unsigned int steps = img.step;
  	unsigned int left_limit = steps/3;
  	unsigned int forward_limit = 2*(steps/3);
  
  	unsigned int left_side =0;
  	unsigned int right_side =0;
  	unsigned int mid_point = 0;
  	unsigned int pixel_count =0;
  	double white_ratio = 0;
  	
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
  
  
  	for (int i=0;i<rows;i++){
    		for(int j=0;j<steps;j++){
        		if(img.data[i*steps+j] == white_pixel){
                if (left_side == 0 || j < left_side)
                  left_side = j;
                else if (j >= right_side)
                  right_side = j;
               	
                pixel_count++;
            	}
        	}
     }

  	white_ratio = pixel_count/(steps*rows);
  	mid_point = (right_side - left_side)/2;
  
  	if( (pixel_count > 0) && (white_ratio < 0.45)){
    	if(right_side < left_limit){
        ang_z = 0.3*(steps/2 - mid_point);
        }
      	else if (right_side < forward_limit){
        	if(left_side > left_limit){
            	lin_x = 0.5;
            }
          	else{
              	ang_z = 0.3*(steps/2 - (left_limit-left_side));
            }
        }
      	else{
          if (mid_point > steps/2)
            ang_z = -0.3*(mid_point -steps/2);
          else
            ang_z = 0.1*(-mid_point +steps/2);
        }
    }
  
    drive_robot(lin_x,ang_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
