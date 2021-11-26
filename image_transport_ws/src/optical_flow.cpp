
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
#include <sys/stat.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
// #include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <stdio.h>

#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

using namespace cv;
using namespace std;
double pi = M_PI;


class TurtlebotTeleop {
    public:
    TurtlebotTeleop();
    void keyLoop();
    void watchdog();

    private:
    ros::NodeHandle nh_, ph_;
    double linear_, angular_;
    ros::Time first_publish_;
    ros::Time last_publish_;
    double l_scale_, a_scale_;
    ros::Publisher vel_pub_;
    void publish(double, double);
    boost::mutex publish_mutex_;
};

TurtlebotTeleop::TurtlebotTeleop():
    ph_("~"),
    linear_(0),
    angular_(0),
    l_scale_(1.0),
    a_scale_(1.0)
{
    ph_.param("scale_angular", a_scale_, a_scale_);
    ph_.param("scale_linear", l_scale_, l_scale_);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>(
        "cmd_vel", 1
    );
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig){
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}



int main(int argc, char** argv){
    ros::init(argc, argv, "optical_flow");

    // int my_result = 0;
    // my_result = optical_flow_func();

    TurtlebotTeleop turtlebot_teleop;
    ros::NodeHandle n;
    signal(SIGINT, quit);
    boost::thread my_thread(boost::bind(&TurtlebotTeleop::keyLoop, &turtlebot_teleop));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&TurtlebotTeleop::watchdog, &turtlebot_teleop));
    ros::spin();
    my_thread.interrupt();
    my_thread.join();

    
    return(0);
}

void TurtlebotTeleop::watchdog(){
    boost::mutex::scoped_lock lock(publish_mutex_);
    if((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && (ros::Time::now() > first_publish_ + ros::Duration(0.50))){
        publish(0,0);
    }
}


void imageCallback(const sensor_msgs::Image::ConstPtr &msg){

    cv_bridge::CvImagePtr cv_ptr;
    try{
        // cv::imshow("Mission #1", cv_bridge::toCvShare(msg, "bgr8")-> image);
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cannot convert");
    }
    cv::imshow("Mission #1", cv_ptr->image);
    
    cv::waitKey(1);
}

void TurtlebotTeleop::keyLoop(){
   
      
    int result = 0;
    cv::VideoCapture vc(0);
    // if(!vc.isOpened()) return -1;//Failed to Connect
    vc.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    vc.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cv::Mat frame1, prvs;
    vc >> frame1;
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);

    while(ros::ok()){
        
        Mat frame2, next;
        vc >> frame2;
        if (frame2.empty())
            break;
        cvtColor(frame2, next, COLOR_BGR2GRAY);
        Mat flow(prvs.size(), CV_32FC2);
        calcOpticalFlowFarneback(prvs, next, flow, 0.5, 3, 15, 3, 5, 1.2, 0);
        Mat flow_parts[2];
        split(flow, flow_parts);
        Mat magnitude, angle, magn_norm;
        cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
        normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
		Mat oang = angle * 3.141592 / 180.0;
        angle *= ((1.f / 360.f) * (180.f / 255.f));

        //build hsv image
        Mat _hsv[3], hsv, hsv8, bgr;
        _hsv[0] = angle;
        _hsv[1] = Mat::ones(angle.size(), CV_32F);
        _hsv[2] = magn_norm;
        merge(_hsv, 3, hsv);
        hsv.convertTo(hsv8, CV_8U, 255.0);
        cvtColor(hsv8, bgr, COLOR_HSV2BGR);
        // imshow("Mission #2", bgr);

        int step = 10;
        int number[4] = {};
        //right, left, up, down
        Mat img_vec = frame2;
		for (int r=0; r<angle.rows; r+=step) {
			for (int c=0; c<angle.cols; c+=step){
				float ang = oang.at<float>(r,c);
				float m = magn_norm.at<float>(r,c) * 20.0;
				Point pt1 = cv::Point(c, r);
				Point pt2 = cv::Point(c + m * cos(ang) , r + m * sin(ang));
				arrowedLine(img_vec, pt1, pt2, Scalar(0, 255, 0), 1, 8, 0); 

                if((ang >= 0 && ang < pi/4) || (ang >= (pi/4)*7 && ang < pi*2)){
                    number[0] += 100*m;
                }
                else if(ang >= pi/4 && ang < (pi/4)*3){
                    number[2] += 10*m;
                }
                else if(ang >= (pi/4)*3 && ang < (pi/4)*5){
                    number[1] += 10*m;
                }
                else if(ang >= (pi/4)*5 && ang < (pi/4)*7){
                    number[3] += 10*m;
                }
			}
		}
        int max = number[0];
        for (int i=0;i<4;i++){
            if(max<number[i]){
                max = number[i];
            }
        }
        // cout << max << endl;
        if (max == number[0]){
            cout << "left" << endl;
            // cout << result << endl
            result = 0;
        } 
        else if (max == number[1]){
            cout << "right" << endl;
            result = 1;
        }
        else if (max == number[2]){
            cout << "down" << endl;
            result = 2;
        }
        else if (max == number[3]){
            cout << "up" << endl;
            result = 3;
        }
        imshow("Mission #3", img_vec);
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;


        prvs = next;
        linear_ = angular_ = 0;
        switch (result)
        {
        case 0:
            // cout << "case 0" << endl;
            //ROS_DEBUG("RIGHT");
            angular_ = -10.0;
            break;
        case 1:
            // cout << "case 1" << endl;
            //ROS_DEBUG("LEFT");
            angular_ = 10.0;
            break;
        case 2:
            // cout << "case 2" << endl;
            //ROS_DEBUG("UP");
            linear_ = 10.0;
            break;
        case 3:
            // cout << "case 3" << endl;
            //ROS_DEBUG("DOWN");
            linear_ = -10.0;
            break;
        }
    boost::mutex::scoped_lock lock(publish_mutex_);
    if(ros::Time::now() > last_publish_ + ros::Duration(1.0)){
        first_publish_ = ros::Time::now();
    }
    last_publish_ = ros::Time::now();
    publish(angular_, linear_);
    }
    return ;
    
}

void TurtlebotTeleop::publish(double angular, double linear){
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;
    vel_pub_.publish(vel);

    return;
}

