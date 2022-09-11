#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sys/stat.h>

using namespace std;
using namespace cv;

string name;
int num;


void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        imshow("view", img);

        // writing the image to a defined location as JPEG
        string path = "./" + name + "/" + to_string(num++) + ".jpg";
        bool check = imwrite(path, img);
        cout << "saved: " << path << endl;
        waitKey(3000);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void clickCB(int event, int x, int y, int flags, void* userdata) {
    if  ( event == EVENT_LBUTTONDOWN )
    {
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_images_node");

    cout << "Lego name: ";
    cin >> name; 
    cout << endl;
    num = 0;

    mkdir(name.c_str(), 0777);

    ros::NodeHandle nh;
    namedWindow("view");


    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subColor = it.subscribe("/camera/Scene/image_raw", 1, imageCallback);
    ros::spin();
    destroyWindow("view");
}