#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>

void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

	//ROS_INFO("callback :)");

	int width = msg->width;
	int height = msg->height;

	int d = msg->data[0];
	//int size = *(&msg->data + 1) - msg->data;

	ROS_INFO("width: %d", width);
	ROS_INFO("height: %d", height);
	ROS_INFO("distance %d", d);
	//ROS_INFO("size: %d", size);	

	/*float* depths = (float*)(&msg->data[0]);

	int u = msg->width/2;
	int v = msg->height/2;

	int centerIdx = u + msg->width * v;

	ROS_INFO("Center distance: %g m", depths[centerIdx]);*/
}



int main (int argc, char** argv){

	std::cout << "Starting camera depth" << std::endl;

	ros::init(argc, argv, "depth_node");

	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 10, depthCallback);

	ros::spin();

	return 0;

}
