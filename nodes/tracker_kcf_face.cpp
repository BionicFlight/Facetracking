//make by kirito jin
//@ZJUT e-mail:17758373920@163.com
//2021年09月06日18:28:55
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>
#include "kcftracker.hpp"
#include "fv_tracking/Diff.h"
#include "derror.h"

using namespace std;
using namespace cv;


#define MARKER_SIZE 0.18
#define F1 300
#define F2 300
#define C1 320
#define C2 240

static const std::string RGB_WINDOW = "RGB Image window";

//! Camera related parameters.
int frameWidth_;
int frameHeight_;


float get_ros_time(ros::Time begin);                                                 //获取ros当前时间
cv::Rect selectRect;
cv::Rect result;
std_msgs::Header imageHeader_;
cv::Mat camImageCopy_;
boost::shared_mutex mutexImageCallback_;
boost::shared_mutex mutexImageStatus_;
int recflag = 0;

DERROR derrorX, derrorY;


bool imageStatus_ = false;
bool rec_flag = false;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool jiance = false;

cv::Scalar colors[]=
{
	CV_RGB(255, 0, 0),
	CV_RGB(0, 0, 255),
	CV_RGB(0, 255, 0),
	CV_RGB(255, 255, 0),           //宏定义颜色;
	CV_RGB(255, 0, 255),
	CV_RGB(0, 255, 255),
	CV_RGB(255, 120, 255),
};

void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("[EllipseDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imageHeader_ = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cam_image) 
    {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
        }
        frameWidth_ = cam_image->image.size().width;
        frameHeight_ = cam_image->image.size().height;
    }
    return;
}


vector<Rect> kirito;  
vector<Rect> xipeng;
vector<Rect> tod;
string facedetector_weijie = "/home/kirito/Documents/train_weijie/classifier/cascade.xml";
string facedetector_xipeng = "/home/kirito/Documents/train_xipeng/classifier/cascade.xml";
string facedetector_tod = "/home/kirito/Documents/train_tod/classifierLBP/cascade.xml";
string facedetector_jiaren = "/home/kirito/Documents/train_jiaren/classifier/cascade.xml";
string facedetector_liwen = "/home/kirito/Documents/train_liwen/classifier/cascade.xml";

cv::Mat image_src, image_gray, image_sample;  

void drawText(Mat & image)
{

    CascadeClassifier cascade_1, cascade_2, cascade_3;
    cascade_1.load(facedetector_liwen);

    image_src = image;
    cvtColor(image_src, image_gray, CV_BGR2GRAY);
    equalizeHist(image_gray, image_gray);//直方图均衡化方便识别
    cascade_1.detectMultiScale(image_gray, kirito, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(64, 64));//检测
    for (int i =0; i<kirito.size(); i++)
    {
	cv::rectangle(image_src, kirito[i], Scalar(255, 255, 0), 2);
	//putText(image_src,format("WeiJie"),kirito[i].tl(), FONT_HERSHEY_PLAIN, 1.0,Scalar(255, 255, 0), 2, 10);

	selectRect.x = kirito[i].tl().x;        
	selectRect.y = kirito[i].tl().y;
	selectRect.width = (kirito[i].tl().x-kirito[i].br().x);   
	selectRect.height = (kirito[i].tl().y-kirito[i].br().y);
	selectRect &= cv::Rect(0, 0, frameWidth_, frameHeight_);
	selectRect = cv::Rect(kirito[i].tl().x, kirito[i].tl().y, abs(kirito[i].br().x-kirito[i].tl().x), abs(kirito[i].br().y-kirito[i].tl().y));

        if (selectRect.width*selectRect.height < 64)
        {
          ;
        }
        else
        {
            rec_flag = false;
            bRenewROI = true;    
        } 
    } 
   //cout<< "people number is : " << kirito.size()<<endl;
    if (kirito.size()==1)
    {
       bRenewROI=true;
       jiance=true;
    }
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
    return imageStatus_;
}

//! ROS subscriber and publisher.
image_transport::Subscriber imageSubscriber_;
ros::Publisher pose_pub;

float get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// 创建KCF追踪
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

//日志数据保存
ofstream file;
ofstream file1;


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "tracker_ros");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);
    
    // 接收图像的话题
    imageSubscriber_ = it.subscribe("/camera/rgb/image_raw", 1, cameraCallback);

 
    ros::Publisher flag_version_pub=nh.advertise<geometry_msgs::Point>("/relative_position_flag",10);
    // diff 
    ros::Publisher position_diff_pub=nh.advertise<fv_tracking::Diff>("/position_diff",10);    
  
    sensor_msgs::ImagePtr msg_ellipse;

    const auto wait_duration = std::chrono::milliseconds(2000);

    cv::namedWindow(RGB_WINDOW);
    //cv::setMouseCallback(RGB_WINDOW, onMouse, 0);
  

  float cur_time;
  float last_time;
  float last_error_x,last_error_y;
  float dt;
  float unfilter_vely,unfilter_velx;

  VideoWriter writer("/home/kirito/Documents/mav_ros/VideoTest.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, Size(640, 360));

  geometry_msgs::Point flag_vision;
  fv_tracking::Diff error_pixels;
  ros::Time begin_time = ros::Time::now();

  
  file.open("/home/kirito/Documents/mav_ros/tracker_kcf.txt", ios::binary | ios::app | ios::in | ios::out);
  file1.open("/home/kirito/Documents/mav_ros/tracker_kcf_date.txt", ios::binary | ios::app | ios::in | ios::out);
  file<<"cur_time"<<"  "<<"dt"<<"  "<<"error_pixels.x"<<"  "<<"error_pixels.y"<<"  "<<"error_pixels.velx"<<"  "<<"error_pixels.vely"<<"  "<<"unfilter_velx"<<"  "<<"unfilter_vely"<<"  "<<"error_pixels.Ix"<<"  "<<"error_pixels.Iy"<<"  "<<"error_pixels.recsize"<<"  "<<"error_pixels.selectrec"<<"\n";
  file1<<"cur_time"<<"  "<<"dt"<<"  "<<"kcf.x"<<" "<<"kcf.y"<<" "<<"kcf.width"<<" "<<"kcf.height"<<"\n";

    while (ros::ok())
    {
        cur_time = get_ros_time(begin_time);
        dt = (cur_time - last_time);
        
	if(dt>1.0 || dt<0.0)
	{
	dt=0.05;
	}
	
        while (!getImageStatus()) 
        {
            printf("Waiting for image.\n");
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }

        Mat frame;
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            frame = camImageCopy_.clone();
        }
        
static bool need_tracking_det = false;
        
        if(recflag==0)
        {        
        drawText(frame);  
        cout<< selectRect.x<<"," << selectRect.y<<","<< selectRect.width <<","<<selectRect.height<<"."<<endl;     
        }
        
        if(bRenewROI)
        {
        
        tracker.init(selectRect, frame);
        cv::rectangle(frame, selectRect, cv::Scalar(255, 255, 255), 2, 8, 0);
        putText(image_src,format("liwen"),selectRect.tl(), FONT_HERSHEY_PLAIN, 1.0,Scalar(255, 255, 0), 2, 10);
        bRenewROI = false;
        bBeginKCF = true;
        }
        
	char c = waitKey(5);
		if(c == 27)
		{
		 bBeginKCF = false;
		}
        else if (bBeginKCF)
        {
        
	result = tracker.update(frame);
	flag_vision.x = 0x01;
	error_pixels.x = result.x + result.width/2 - frame.cols/2;
	error_pixels.y = result.y + result.height/2 - frame.rows/2;

	error_pixels.recsize = result.width*result.height;
	error_pixels.selectrec = selectRect.width*selectRect.height;
	
	

float error_x = error_pixels.x;
float error_y = error_pixels.y;

        derrorX.add_error(error_x, cur_time);
        derrorY.add_error(error_y, cur_time);
        derrorX.derror_output();
        derrorY.derror_output();
        derrorX.show_error();
        derrorY.show_error();

        error_pixels.velx=derrorX.Output;
        error_pixels.vely=derrorY.Output;

        error_pixels.Ix += error_pixels.x*dt;
        error_pixels.Iy += error_pixels.y*dt;

        unfilter_velx = (error_pixels.x-last_error_x)/dt;
        unfilter_vely = (error_pixels.y-last_error_y)/dt;

        last_time = cur_time;
        last_error_x = error_pixels.x;
        last_error_y = error_pixels.y;
        cv::rectangle(frame, result, cv::Scalar(255, 0, 0), 2, 8, 0);
        putText(image_src,format("liwen"),result.tl(), FONT_HERSHEY_PLAIN, 1.0,Scalar(255, 255, 0), 2, 10);
	    cout<< result.x<<"," << result.y<<","<< result.width <<","<<result.height<<"."<<endl;
        file1<<cur_time<<"  "<<dt<<"  "<<result.x<<"  "<<result.y<<" "<<result.width<<" "<<result.height<<"\n";
        }

       else
      {
        flag_vision.x = 0x00;
        error_pixels.x=0.0;
        error_pixels.y=0.0;
        error_pixels.Ix=0.0;
        error_pixels.Iy=0.0;
        error_pixels.velx=0.0;
        error_pixels.vely=0.0;
        error_pixels.recsize=0.0;
        error_pixels.selectrec=0.0;
      }

     file<<cur_time<<"  "<<dt<<"  "<<error_pixels.x<<"  "<<error_pixels.y<<"  "<<error_pixels.velx<<"  "<<error_pixels.vely<<"  "<<unfilter_velx<<"  "<<unfilter_vely<<"  "<<error_pixels.Ix<<"  "<<error_pixels.Iy<<"  "<<error_pixels.recsize<<"  "<<error_pixels.selectrec<<"\n";
    position_diff_pub.publish(error_pixels);
    flag_version_pub.publish(flag_vision);

float left_point=frame.cols/2-20;
float right_point=frame.cols/2+20;
float up_point=frame.rows/2+20;
float down_point=frame.rows/2-20;
	//draw
	line(frame,Point(left_point,frame.rows/2),Point(right_point,frame.rows/2),Scalar(0,255,0),1,8);
	line(frame,Point(frame.cols/2,down_point),Point(frame.cols/2,up_point),Scalar(0,255,0),1,8);
	putText(frame,"x:",Point(50,60),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),3,8);
	putText(frame,"y:",Point(50,90),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),3,8);

	//draw
	char s[20]="";
	sprintf(s,"%.2f",error_pixels.x);
	putText(frame,s,Point(100,60),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),2,8);
	sprintf(s,"%.2f",error_pixels.y);
	putText(frame,s,Point(100,90),FONT_HERSHEY_SIMPLEX,1,Scalar(255,23,0),2,8);

	imshow(RGB_WINDOW, frame);
	writer << frame;
	waitKey(5);
        

        ros::spinOnce();
        loop_rate.sleep();
    }
    file.close();
    file1.close();
}




