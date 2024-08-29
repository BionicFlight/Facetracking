//Capture Camera View---advanced by Weijie Jin -----------------------------------------
//2021年09月06日18:31:55.
// email: 17758373920@163.com
//wechat: xiaofuxing

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include "id_management.h"

using namespace std;
using namespace cv;


image_transport::Publisher image_pub;
//设置图像大小
cv::Size image_size = Size(640, 360);

int main(int argc, char **argv)
{
    int cap_flag=0;
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(30);
    // 在这里修改发布话题名称
    image_pub = it.advertise("/camera/rgb/image_raw", 1);
    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改
    cv::VideoCapture cap(4);

    // 设置摄像头分辨率
     cap.set(CAP_PROP_FRAME_WIDTH, 640);
     cap.set(CAP_PROP_FRAME_HEIGHT, 360);



    cv::Mat frame;
    // 设置全屏
    // namedWindow("web_cam frame", CV_WINDOW_NORMAL);
    // setWindowProperty("web_cam frame", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

    sensor_msgs::ImagePtr msg;
    int id = 0;
    while (ros::ok())
  {

         cap >> frame;
        if (!frame.empty())   

        {
            if (0 == id)
            {
                cout << "IM RESOLUTION: " << frame.cols << " x " << frame.rows << "  " << frame.channels() << frame.size << endl;
            }
            //添加ID
            frame = IDManagement::add_id(id, frame);
            id = id+1;
            // 改变图像大小并显示图片
            resize(frame, frame, image_size);
            // imshow("web_cam frame", frame);
            waitKey(5);
            // 设置图像帧格式->bgr8
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // 将图像通过话题发布出去
            image_pub.publish(msg);
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        loop_rate.sleep();
    }
         cap.release();


}
