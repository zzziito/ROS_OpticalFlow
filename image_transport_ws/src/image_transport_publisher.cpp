#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
// using namespace cv;

int main(int argc, char** argv){
    if(argv[1] == NULL){
        return 1;
    }

    ros::init(argc, argv, "image_transport_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/image_raw",1);
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;

    if(!(video_sourceCmd >> video_source)){
        ROS_INFO("video sourceCmd");
        return 1;
    }

    cv::VideoCapture cap(video_source);

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;
    ros::Rate loop_rate(5);
    while(nh.ok()){
        cap>>frame;
        if(!frame.empty()){
            msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", frame).toImageMsg();
            pub.publish(msg);
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
}

// static const std::string IMAGE_PATH = "/home/darrenl/lena.jpg";
// static const std::string TOPIC_NAME = "camera/rgb/image";

// int publishImage(std::string filepath)
// {
//     Mat image;
//     image = imread(filepath, CV_LOAD_IMAGE_COLOR);   // Read the file
//     std::cout << "Path " << filepath << std::endl;
//     if(!image.data)                              // Check for invalid input
//     {
//         std::cout << "Could not open or find the image" << std::endl ;
//         return -1;
//     }

//     ros::NodeHandle nh;
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise(TOPIC_NAME, 1);
//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     ros::Rate loop_rate(5);

//     while (nh.ok()) {
//         pub.publish(msg);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// int publishImageWithoutImage_transport()
// {
//     ROS_INFO("Topic : %s", TOPIC_NAME.c_str());
//     ROS_INFO("IMAGE PATH : %s", #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// using namespace cv;

// static const std::string IMAGE_PATH = "/home/darrenl/lena.jpg";
// static const std::string TOPIC_NAME = "camera/rgb/image";

// int publishImage(std::string filepath)
// {
//     Mat image;
//     image = imread(filepath, CV_LOAD_IMAGE_COLOR);   // Read the file
//     std::cout << "Path " << filepath << std::endl;
//     if(!image.data)                              // Check for invalid input
//     {
//         std::cout << "Could not open or find the image" << std::endl ;
//         return -1;
//     }

//     ros::NodeHandle nh;
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher pub = it.advertise(TOPIC_NAME, 1);
//     sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
//     ros::Rate loop_rate(5);

//     while (nh.ok()) {
//         pub.publish(msg);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }
// }

// int publishImageWithoutImage_transport()
// {
//     ROS_INFO("Topic : %s", TOPIC_NAME.c_str());
//     ROS_INFO("IMAGE PATH : %s", IMAGE_PATH.c_str());
//     ros::NodeHandle nh;
//     std::string image_path = IMAGE_PATH;
//     cv_bridge::CvImage cv_image;
//     cv_image.image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
//     cv_image.encoding = "bgr8";
//     sensor_msgs::Image ros_image;
//     cv_image.toImageMsg(ros_image);

//     ros::Publisher pub = nh.advertise<sensor_msgs::Image>(TOPIC_NAME, 1);
//     ros::Rate loop_rate(5);

//     while (nh.ok())
//     {
//         pub.publish(ros_image);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "image_transport_publisher");
//     publishImageWithoutImage_transport();
//     return 0;
// }
//     sensor_msgs::Image ros_image;
//     cv_image.toImageMsg(ros_image);

//     ros::Publisher pub = nh.advertise<sensor_msgs::Image>(TOPIC_NAME, 1);
//     ros::Rate loop_rate(5);

//     while (nh.ok())
//     {
//         pub.publish(ros_image);
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

// }

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "image_transport_publisher");
//     publishImageWithoutImage_transport();
//     return 0;
// }

// #include <ros/ros.h>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
// // using namespace cv;

// int main(int argc, char** argv){
//     ros::init(argc, argv, "image_transport_publisher");
//     cv::VideoCapture vc(0);
//     if(!vc.isOpened()) return -1;//Failed to Connect
//     vc.set(CV_CAP_PROP_FRAME_WIDTH, 640);
//     vc.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//     cv::Mat img;

//     while(1){
//         vc >> img;
//         if(img.empty()) break;
//         cv::imshow("Mission #1", img);
//         if(cv::waitKey(10)==27) break; //ESC
//     }
//     cv::destroyAllWindows();
//     return 0;
// }