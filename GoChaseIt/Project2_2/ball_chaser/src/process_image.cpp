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
	int pixel_count=0;
	float ratio;
	bool flag = false;

	enum Pos : int {LEFT, FORWARD, RIGHT, IDLE} pos = IDLE;

	  
	for (int i = 0; i < img.height * img.step; i+=3) {
	    
		    if (img.data[i] == white_pixel && img.data[i+1] == white_pixel 
						   && img.data[i+2] == white_pixel) 
		   {
			      int col = i % img.step;
			      if (col < img.step * 0.33) {
				if(!flag){				
					pos = LEFT;
					flag=true;
				}
			      } 
			      else if (col > img.step * 0.66) {
				if(!flag){				
					pos = RIGHT;
					flag=true;
				}
			      } 
			      else {
				if(!flag){
				pos = FORWARD;
				flag=true;
				}
			      }
			      pixel_count +=3; 
		    }
	  }
	
	ratio = pixel_count/(img.height*img.step);
	if(ratio > 0.3)
		pos = IDLE;

	
	if(pos == LEFT)
		ang_z = 0.3;
	else if(pos == RIGHT)
		ang_z = -0.3;
	else if(pos == FORWARD)
		lin_x = 0.5;
	else if (pos == IDLE){
		ang_z = 0;
		lin_x = 0;	
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
