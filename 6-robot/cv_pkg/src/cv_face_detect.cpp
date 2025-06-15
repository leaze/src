#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>         // 格式转换器
#include <sensor_msgs/image_encodings.h> // 图像格式编码
#include <opencv2/imgproc/imgproc.hpp>   // opencv图像处理头文件
#include <opencv2/highgui/highgui.hpp>   // opencv界面显示头文件
#include <geometry_msgs/Twist.h>
#include <opencv2/objdetect/objdetect.hpp>
#include <image_transport/image_transport.h>

using namespace cv;

static CascadeClassifier face_cascade;

static Mat frame_gray;
static std::vector<Rect> faces;
static std::vector<cv::Rect>::const_iterator face_iter;

void callbackRGB(const sensor_msgs::Image msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal = cv_ptr->image;

    // change contrast: 0.5 = half  ; 2.0 = double
    imgOriginal.convertTo(frame_gray, -1, 1.5, 0);

    // create B&W image
    cvtColor( frame_gray, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 9, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

    if(faces.size() > 0)
    {
        std::vector<cv::Rect>::const_iterator i;
        for (face_iter = faces.begin(); face_iter != faces.end(); ++face_iter) 
        {
            cv::rectangle(
                imgOriginal,
                cv::Point(face_iter->x , face_iter->y),
                cv::Point(face_iter->x + face_iter->width, face_iter->y + face_iter->height),
                CV_RGB(255, 0 , 255),
                2);
        }
    }
    imshow("faces", imgOriginal);
    waitKey(1);
}


int main(int argc, char **argv)
{
    cv::namedWindow("faces");
    ros::init(argc, argv, "demo_cv_face_detect");

    ROS_INFO("demo_cv_face_detect");

    std::string strLoadFile;
    char const* home = getenv("HOME");
    std::cout << "home: " << home << std::endl;
    strLoadFile = home;
    strLoadFile += "/code/catkin_ws";    //工作空间目录
    strLoadFile += "/src/wpr_simulation/config/haarcascade_frontalface_alt.xml";
    std::cout << "strLoadFile: " << strLoadFile << std::endl;
    bool res = face_cascade.load(strLoadFile);
	if (res == false)
	{
		ROS_ERROR("fail to load haarcascade_frontalface_alt.xml");
        return 0;
	}
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/kinect2/qhd/image_color_rect", 1 , callbackRGB);

    ros::spin();
    return 0;
}
// roslaunch wpr_simulation wpr1_single_face.launch
// rosrun cv_pkg cv_face_detect
// rosrun wpr_simulation keyboard_vel_ctrl