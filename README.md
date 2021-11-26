# ROS_OpticalFlow

_경희대학교 2021-2 로봇프로그래밍 수업 중간대체과제입니다._

## 1. Package Manual

완벽하게 세 번째 미션을 수행하지는 못 하여 두 가지 버전으로 제출합니다. 

#### 1. uvc_camera 에서 영상을 받아오지만 optical flow 가 조금 부정확한 코드

실행 방법

```
$ rosrun uvc_camera uvc_camera_node
$ rosrun image_transport_ws optical_flow_car
```

#### 2. web cam 에서 영상을 받아오지만 optical flow 와 gazebo 가 완벽하게 작동하는 코드

실행 방법

(uvc_camera 가 작동하고 있지 않은 상태에서)

```
$ rosrun image_transport_ws optical_flow
```


## 2. Making Process

#### 1. Mission 1

> Display image in real-time using imshow

[미션#1 동영상 링크](https://youtu.be/2pkJh2Ftb_8)

![](https://images.velog.io/images/zzziito/post/fc89b05b-ec3a-4bd6-92e8-2032b0b3153f/uvc_camera_rqt_graph.png)

uvc_camera 에서 publish 하는 image_raw 를 subscribe 하여 imshow 로 보여주는 코드입니다. 
Node Graph 를 통해 uvc_camera 에서 image_raw 를 받아오고 있다는 것을 볼 수 있습니다. 

**실행 코드**
```
$ rosrun uvc_camera uvc_camera_node
$ rosrun image_transport_ws image_publisher
```

**소스 코드**
```c

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::Image::ConstPtr &msg){

    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cannot convert");
    }
    cv::imshow("Mission #1", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("/image_raw", 1,
            imageCallback);
    return 0;
}
```


#### 2. Mission 2

> Display input image with motion vectors

이때 uvc_camera 를 통해 받아온 영상은 optical flow 가 조금 미흡하게 시행되었습니다. 

[uvc_camera 로 시행한 mission#2 동영상](https://youtu.be/PAiveu00zxE)

![](https://images.velog.io/images/zzziito/post/b2e1ef4c-31da-47bf-943d-161eb6c5f5c3/image.png)

cv::VideoCapture vc(0) - 웹캠 - 으로 받아온 영상으로 한 optical flow 는 잘 작동하여 아래 gazebo 미션은 웹캠으로 수행하였습니다. 

[미션#2 동영상 링크](https://youtu.be/7kZvf4LSWxs)

![](https://images.velog.io/images/zzziito/post/17e43fe2-a11e-4af7-9060-a83568ee1c7a/image.png)





#### 3. Mission 3

> Get Major direction

[미션#3 동영상 링크](https://youtu.be/p4zrdl24DgQ)

**trial #1**

물체 움직임의 평균적인 방향을 추정하기 위해, 처음에는 다음과 같은 방법을 사용했습니다. 

```c
if((ang >= 0 && ang < pi/4) || (ang >= (pi/4)*7 && ang < pi*2)){
                    number[0] += 1;
                }
                else if(ang >= pi/4 && ang < (pi/4)*3){
                    number[2] += 1;
                }
                else if(ang >= (pi/4)*3 && ang < (pi/4)*5){
                    number[1] += 1;
                }
                else if(ang >= (pi/4)*5 && ang < (pi/4)*7){
                    number[3] += 1;
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
switch (result)
        {
        case 0:
            angular_ = -10.0;
            break;
        case 1:
            angular_ = 10.0;
            break;
        case 2:
            linear_ = 10.0;
            break;
        case 3:
            linear_ = -10.0;
            break;
        }
```
![](https://images.velog.io/images/zzziito/post/eb917f7f-4148-4bbd-992e-6b2f8fcead40/image.png)

ang 변수에 들어있는 벡터의 각도에 따라 number 라는 행렬의 요소를 1씩 더했습니다. 
이후 number 행렬의 가장 큰 값을 검사해 up / down / right / left 를 판단했습니다. 

[불충분한 알고리즘으로 인해 진행값이 자주 바뀌는 터틀봇 영상](https://youtu.be/6uaRQeNOyTs)

![](https://images.velog.io/images/zzziito/post/ceb75214-df5d-4528-896c-a9ffa3b32827/%EC%8A%A4%ED%81%AC%EB%A6%B0%EC%83%B7%202021-11-22%20%EC%98%A4%ED%9B%84%209.21.17.png)

하지만 동영상에서 보이듯이, 벡터의 크기를 고려하지 않아 값이 부정확하게 나왔습니다. 또한 결과값이 일정하지 않아 터틀봇 또한 진동하는 듯한 모습을 볼 수 있었습니다. 

**trial #2**

```
number[0] += 100*m
```

이와 같이 벡터의 크기를 고려해 주니, 이전보다는 일관되게 값이 나오는 것을 확인할 수 있었습니다. 

#### Send message to Gazebo
 
turtlebot__teleop 소스코드를 살펴보면, TurtlebotTeleop 클래스의 keyLoop() 함수에서 switch 문에 따라 angular_, linear_ 값을 결정하고, 이를 publish 함수에서 publish 한다는 것을 알 수 있습니다. 

따라서 저는 전체 이미지 처리 코드를 keyLoop 함수에 넣어서 결과값을 cmd_vel / geometry_msgs / twist 형태로 publish 하도록 하였습니다. 




## 3. Further Development (Application Example)


과제를 통해 ROS 를 이용하면 달리 통신을 따로 구축하지 않아도 같은 MASTER 에 접속한 기기끼리는 쉽게 메시지를 주고 받을 수 있다는 것을 알게 되었습니다. 
이러한 큰 이점을 실제로 경험해보고자, Gazebo 가 아니라 실제 미니 자동차를 만들어서 메시지를 날려 동작시켜 보았습니다. 

### 결과 동영상

![](https://images.velog.io/images/zzziito/post/19ce3caf-1777-4c69-8a30-6056ca187464/image.png)

[바퀴 돌아가는 차 영상 링크](https://youtu.be/6iFWkMA6cug)

Optical Flow 로 나온 결과값에 따라서 바퀴가 전,후,좌,우로 동작하는 것을 볼 수 있었습니다. 

#### 사용한 기기

- Jetson Nano 4GB
- L298N motor driver
- PCA9685

**diagram**

![](https://images.velog.io/images/zzziito/post/4f5a0cee-4a60-4f01-bacc-30b100d16ffb/l298n.png)

![](https://images.velog.io/images/zzziito/post/0a38b0f1-9d15-458a-96b4-50893f63d9df/IMG_7987.jpg)
이 구성도에서 라즈베리파이만 젯슨 나노로 변경하여 사용했습니다. 
젯슨 나노에 기존과 똑같이 ROS 를 세팅하고, 수업 때 실습한 topic_exam 에 기반한 패키지를 만들었습니다. 

이때 slave 인 jetson nano 를 ROS_MASTER 에 연결해 주기 위해서, 
~/.bashrc 를 다음과 같이 수정해 주었습니다. 

**master 컴퓨터의 .bashrc**

```
export ROS_IP = 192.168.0.23
export ROS_MASTER_URI = http://192.168.0.23:11311
export ROS_HOSTNAME = $ROS_IP
```

**slave 컴퓨터의 .bashrc**

``` 
export ROS_MASTER_URI = http://192.168.0.23:11311 
export ROS_HOSTNAME = 192.168.0.73
```

[기기 간 메시지 주고 받는 화면](https://youtu.be/fvawyRPruvw)

다음과 같이 Master 에 잘 연결된 것을 볼 수 있습니다. 
