
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
#include <sensor_msgs/image_encodings.h>
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

static const std::string OPENCV_WINDOW = "Mission #2";

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

void TurtlebotTeleop::watchdog(){
    boost::mutex::scoped_lock lock(publish_mutex_);
    if((ros::Time::now() > last_publish_ + ros::Duration(0.15)) && (ros::Time::now() > first_publish_ + ros::Duration(0.50))){
        publish(0,0);
    }
}


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  //cv::Mat capture, current, previous, flow, visual_flow, opening;
  cv::Mat frame1, frame2, prvs, next;
  int result = 0;

public:
  ImageConverter() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/image_raw", 1,
      &ImageConverter::imageCb, this);
    //image_pub_ = it_.advertise("/image_raw", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, 
        sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    frame1 = cv_ptr->image;
    
    cvtColor(frame1, prvs, COLOR_BGR2GRAY);
    if(ros::ok()) 
    {
      frame2 = cv_ptr->image;
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
        imshow(OPENCV_WINDOW, img_vec);
        //imshow("Mission #3", img_vec);
        int keyboard = waitKey(30);

        prvs = next;
        // if (keyboard == 'q' || keyboard == 27)
        //     break;

    //   cv::calcOpticalFlowFarneback(previous, current, 
    //     flow, 0.5, 1, 4, 1, 5, 1.1,3);
    //   opening = current.clone();
      
    //   int flow_ch = flow.channels();
    //   for(int y = 0; y < opening.rows; ++y)
    //   {
    //     float* psrc = (float*)(flow.data + flow.step * y);        
	// 			for(int x = 0; x < opening.cols; ++x)
    //     {
	// 			  float dx = psrc[0];
    //       float dy = psrc[1];
	// 			  float r = (dx + dy);
	// 				if( 4 < r )
	// 				  opening.data[ y * opening.step + x  ] = 0;
    //       else
	// 				  opening.data[ y * opening.step + x  ] = 255;
	// 				psrc += flow_ch;
	// 			}
	// 		}
	// 		cv::morphologyEx(
    //      opening,
    //      opening,
    //      cv::MORPH_CLOSE,        
    //      cv::Mat(),
    //      cv::Point( -1, -1 ),
    //      3,                     
    //      cv::BORDER_CONSTANT,
    //      cv::morphologyDefaultBorderValue()
    //      );
	// 		//オープニング
    //   cv::morphologyEx(
    //      opening,
    //      opening,
    //      cv::MORPH_OPEN,       
    //      cv::Mat(),
    //      cv::Point( -1, -1 ),
    //      50,                  
    //      cv::BORDER_CONSTANT,
    //      cv::morphologyDefaultBorderValue()
    //      );

    //   int x,y;
    //   int s_x,s_y,g_x,g_y;
    //   bool bEnd = false;

    //   for(y = 0; y < opening.rows; ++y)
    //   {      
	// 			for(x = 0; x < opening.cols; ++x)
    //     {
	// 			   if( opening.data[ y * opening.step + x ] == 0 )
    //        {
	// 			     s_y = y;
	// 			     bEnd = true;
	// 			   }
	// 			   if( bEnd ) 
    //          break; 
	// 			}
	// 			if( bEnd ) 
    //       break; 
	// 		}
	// 		bEnd = false;    
	// 		for(x = 0; x < opening.cols; ++x)
    //   {
	// 		  for(y = 0; y < opening.rows; ++y)
    //     { 
	// 			   if( opening.data[ y * opening.step + x ] == 0 )
    //        {
	// 			     s_x = x;
	// 			     bEnd = true;
	// 			   }
	// 			   if( bEnd )
    //          break;
	// 			}
	// 			if( bEnd )
    //       break;
	// 		}
	// 		bEnd = false; 

	// 		for( y = opening.rows-1; -1 < y; --y)
    //   {      
	// 			for( x = opening.cols-1; -1 < x; --x)
    //     {
	// 			   if( opening.data[ y * opening.step + x ] == 0 )
    //        {
	// 			     g_y = y;
	// 			     bEnd = true;
	// 			   }
	// 			   if( bEnd )
    //          break;
	// 			}
	// 			if( bEnd )
    //       break;
	// 		}   
	// 		bEnd = false; 
	// 		for( x = opening.cols-1; -1 < x; --x)
    //   {
	// 			for( y = opening.rows-1; -1 < y; --y)
    //     { 
	// 			   if( opening.data[ y * opening.step + x ] == 0 )
    //        {
	// 			     g_x = x;
	// 			     bEnd = true;
	// 			   }
	// 			   if( bEnd )
    //          break;
	// 			}
	// 			if( bEnd )
    //       break;
	// 		}
    //   bEnd = false; 

    //   visual_flow = capture.clone();

	// 		cv::rectangle(visual_flow, cv::Point(s_x,s_y), 
    //     cv::Point(g_x, g_y), cv::Scalar(0,0,255), 3, 4);
      
    //   cv::imshow(OPENCV_WINDOW, visual_flow);

    }

    // previous = current.clone();
    // cv::waitKey(3);

    // image_pub_.publish(cv_ptr->toImageMsg()); 
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

void TurtlebotTeleop::publish(double angular, double linear){
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale_*angular;
    vel.linear.x = l_scale_*linear;
    vel_pub_.publish(vel);

    return;
}