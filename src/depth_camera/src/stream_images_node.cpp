#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/matx.hpp>


#include <cmath>
#include <algorithm>    // std::min_element


using namespace std;
using namespace cv;


double centerX = 320, centerY = 240;
int minIndex = 0;   // contorno più vicino al centro
Point2f nearestCenter;  // centro
ros::Publisher angle_pub;
ros::Publisher pos_pub;

//get distance from the center of the image
double getDistance(Point2f p) {
    return sqrt((p.x-centerX)*(p.x-centerX)+(p.y-centerY)*(p.y-centerY));
}
double getDistance(Point2f p, Point2f p2) {
    return sqrt((p.x-p2.x)*(p.x-p2.x)+(p.y-p2.y)*(p.y-p2.y));
}
bool compP(Point i, Point j) { 
    return getDistance(i, nearestCenter) < getDistance(j, nearestCenter); 
}
void pointsCallback(const sensor_msgs::ImageConstPtr &msg)
{
    minIndex = 0;
    try
    {
        Mat m = cv_bridge::toCvShare(msg)->image;
        Mat m2, m3;
        // 1 -> yes, 0 --> on the side, -1 -> down
        int upward = 1;

        //float ground = m.at<float>(0, 0);
        double ground;
        minMaxLoc(m, NULL, &ground, NULL, NULL);
        // set tresh a bit higher than ground, so it's fluttuazione-safe
        threshold( m, m2, ground-0.015, 255, 4 );
        threshold( m2, m3, (0.001), 1, 0 );
        
        vector<Vec4i> hierarchy;
        vector<vector<Point>> contours;
        {
            Mat scaled_mat;
            Rect left = Rect(0, 400, 175, 80);
            Rect right = Rect(425, 400, 215, 80);
            rectangle(m3, left, Scalar(0), -1);
            rectangle(m3, right, Scalar(0), -1);

            m3.convertTo(scaled_mat, CV_8UC1);
            // find contours
            findContours( scaled_mat, contours, hierarchy, RETR_CCOMP , CHAIN_APPROX_NONE);
        }
        
        // get the moments
        vector<Moments> mu(contours.size());
        for( int i = 0; i<contours.size(); i++ ) { 
            mu[i] = moments( contours[i], false ); 
        }
        // get the centroid of figures.
        vector<Point2f> mc;
        for( int i = 0; i<contours.size(); i++) { 
            mc.push_back(Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 )); 
        }
        // draw contours
        Mat drawing(m3.size(), CV_8UC3, Scalar(255,255,255));
        {
            vector<double> dist(mc.size());

            upward = (hierarchy.size() < 3) ? 1 : -1;
            
            for( int i = 0; i<mc.size(); i++ ) {
                if (hierarchy.size() < 3) {
                    dist[i] = getDistance(mc[i]);
                    if(dist[i] < dist[minIndex])
                        minIndex = i;
                } else {
                    // consider only contour where child exist ([2] != -1) 
                    // and has no parent ([3] == -1)
                    if(hierarchy[i][2] != -1 && hierarchy[i][2] == -1) {
                        dist[i] = getDistance(mc[i]);
                        if(dist[i] < dist[minIndex])
                            minIndex = i;
                    }
                }
            }
        }        
        
        if (mc.size() > 0) {
            drawContours(drawing, contours, minIndex, Scalar(167,151,0), 2);
            nearestCenter = mc[minIndex];
            
            circle( drawing, nearestCenter, 5, Scalar(0,0, 255), -1);
            RotatedRect minRect;
            minRect = minAreaRect( contours[minIndex] );
            

            Point2f pts[4]; 
            minRect.points(pts);
            double l1 = getDistance(pts[1], pts[0]);
            double l2 = getDistance(pts[0], pts[3]);
            double rotAngle;
            double shortSide;
            double angleRad;
            double ratio;
            if (l1 > l2) {
                //cout << "rotato a sinistra" << endl;
                rotAngle = -minRect.angle;
                shortSide = l2;
                ratio = l1/l2;
            }                
            else {
                //cout << "rotato a destra" << endl;
                rotAngle = -90 - minRect.angle;
                shortSide = l1;
                ratio = l2/l1;
            }
            
            // from degree to rad
            angleRad = rotAngle * M_PI / 180;
            
            // convexity defetcs
            vector<int> hull;
            vector<Vec4i> defects;
            convexHull(contours[minIndex], hull, true);
            convexityDefects(contours[minIndex], hull, defects);

            double offset;
            if ((ratio < 1.90) && (ratio > 1.70)) {
                offset = M_PI/2;
            } else {
                offset = 0;
            }
            
            // -1 -> on the left, 1 -> on the right
            int side = 0;

            if (defects.size() != 0)
            {
                sort(defects.begin(), defects.end(), [contours](Vec4i v1, Vec4i v2){
                    return v1[3] > v2[3];
                });
                if (defects[0][3] > 768) { // on the side
                    upward = 0;
                    vector<Point>::const_iterator it = find_if(contours[minIndex].cbegin(), contours[minIndex].cend(), [angleRad,offset](Point pp)->bool{
                        // y = mx + q
                        // q = LegoCenter.y + LegoCenter.x/tan(-(anglerad+offset))
                        // Y = -X/tan(-(anglerad+offset)) + q
                        // 0 = LegoCenter.y - Y + (-X + LegoCenter.x)/tan(-(anglerad+offset))
                        // checks whether the error 15 (pixel)
                        return abs(nearestCenter.y - pp.y + (-pp.x + nearestCenter.x)/tan(-(angleRad+offset))) < 15 && pp.y < nearestCenter.y;
                    });

                    circle( drawing, contours[minIndex][defects[0][2]], 3, Scalar(0,0,0), -1);
                        
                    float q = nearestCenter.y + nearestCenter.x/tan(-(angleRad+offset));
                    float xx = nearestCenter.x+150;
                    float yy = -xx/tan(-(angleRad+offset)) + q;
                    
                    line(drawing, nearestCenter, Point(xx, yy), Scalar(255,0,255));
                    if (it != contours[minIndex].cend()) {
                        int i = it-contours[minIndex].cbegin();
                        int diff;
                        if (defects[0][2] > i) {
                            diff = defects[0][2] - i;
                            if (diff < contours[minIndex].size()/2)
                                side = -1;
                            else
                                side = 1;
                        }
                        else {
                            diff = i - defects[0][2];
                            if (diff < contours[minIndex].size()/2)
                                side = 1;
                            else
                                side = -1;
                        }

                        circle( drawing, *it, 5, Scalar(0,0,0), -1);
                    }
                }
            }
                
            std_msgs::String msgVal;
            msgVal.data = to_string(angleRad);
            msgVal.data += " ";
            msgVal.data += to_string(upward);
            msgVal.data += " ";
            msgVal.data += to_string(side);
            angle_pub.publish(msgVal);
        }

        line(drawing, Point(centerX,0), Point(centerX,480), Scalar(255, 0, 0));
        line(drawing, Point(0,centerY), Point(640,centerY), Scalar(255, 0, 0));

        imshow( "Contours", drawing );

        waitKey(30);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }
}
void positionreached(const std_msgs::String::ConstPtr &Reached)
{
    std_msgs::String val;
    std::stringstream send;
    double x = nearestCenter.x-centerX, y = nearestCenter.y-centerY;
    send << x << " ";
    send << y;
    val.data = send.str();
    pos_pub.publish(val);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_images_node");
    ros::NodeHandle nh;
    namedWindow( "Contours", WINDOW_AUTOSIZE );

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subDepth = it.subscribe("/camera/depth/image_raw", 1, pointsCallback);
    angle_pub = nh.advertise<std_msgs::String>("stream_images_node/rotation", 1000);
    pos_pub = nh.advertise<std_msgs::String>("stream_images_node/err_pos", 1);

    // in posizione sopra al pezzo
    ros::Subscriber arrived = nh.subscribe<std_msgs::String>("stream_images_node/in_place", 1, positionreached);

    ros::spin();
    //destroyWindow("view");
    destroyWindow("Contours");
}