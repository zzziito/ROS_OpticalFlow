#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){
	ros::init(argc, argv, "topic_exam_publisher");
    ros::NodeHandle n;
    
    ros::Publisher topic_exam_pub = n.advertise<std_msgs::String>("topic_exam_message", 1000);
    ros::Rate loop_rate(10);
    
    int count = 0;
    while(ros::ok()){
    	std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world" << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        topic_exam_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
	return 0;
}
