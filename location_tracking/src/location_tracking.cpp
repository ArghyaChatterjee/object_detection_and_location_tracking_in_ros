#include "ros/ros.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "location_msgs/SingleObjectLocation.h"
#include "location_msgs/MultipleObjectLocation.h"

darknet_ros_msgs::BoundingBox boundingBox; 
darknet_ros_msgs::BoundingBoxes boundingBoxesResults;
std::vector<darknet_ros_msgs::BoundingBox> vec;
location_msgs::MultipleObjectLocation multipleobjectlocation;
ros::Publisher LocationPublisher;

cv_bridge::CvImagePtr cv_ptr;
cv::Mat depthImage(480, 640, CV_16UC1);

// input your realsense camera parameters from camera info topic 
double fx = 616.00;
double fy = 616.00;
double cx = 315.75;
double cy = 238.22;

int x_c = 0;
int y_c = 0;
double dist = 0;
double Xtarget = 0;
double Ytarget = 0;
double Ztarget = 0;

void bb_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    vec = msg->bounding_boxes;

    multipleobjectlocation.header.stamp = ros::Time::now();
    multipleobjectlocation.header.frame_id = "location";

    std::for_each(vec.begin(), vec.end(), [](const darknet_ros_msgs::BoundingBox& x){
        location_msgs::SingleObjectLocation singleobjectlocation;
        std::cout << x << std::endl;
        
        x_c = ((int) x.xmin + (int) x.xmax)/2;
        y_c = ((int) x.ymin + (int) x.ymax)/2;
        dist = depthImage.at<unsigned short>(y_c, x_c);

        singleobjectlocation.Class = x.Class;
        //assuming the distance between rgb image center and camera center is 35
        singleobjectlocation.x = dist*(x_c - cx)/fx - 35; 
        singleobjectlocation.y = dist*(y_c - cy)/fy;
        singleobjectlocation.z = dist;
        multipleobjectlocation.multiple_object_location.push_back(singleobjectlocation);
    });
    LocationPublisher.publish(multipleobjectlocation);
    multipleobjectlocation.multiple_object_location.clear();
}

void dai_callback(const sensor_msgs::Image::ConstPtr& msg){
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    depthImage = cv_ptr->image;
}

/*
void camera_info_callback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    fx = msg->K[0];
    cx = msg->K[2];
    fy = msg->K[4];
    cy = msg->K[5];
    printf("fx : %lf, fy : %lf, cx : %lf, cy : %lf\n", fx, fy, cx, cy);
}
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Location_Tracking");
    ros::NodeHandle nh;
    //ros::Subscriber camera_parameters = n.subscribe("/camera/color/camera_info", 10, camera_info_callback);
    ros::Subscriber depth_aligned_image = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, dai_callback);
    ros::Subscriber bounding_box_inference = nh.subscribe("/darknet_ros/bounding_boxes", 10, bb_callback);
    LocationPublisher = nh.advertise<location_msgs::MultipleObjectLocation>("/object_location", 10);
    
    ros::spin();

    return 0;
}