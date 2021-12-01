#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <Eigen/Core>
#include <iostream>

//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("realsense pcl"));
//ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
//cv::Mat input_img;
//int img_width;
//int img_height;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
   // 声明存储原始数据与滤波后的数据的点云的格式
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // 转化为PCL中的点云的数据格式
  pcl_conversions::toPCL(*input, *cloud);

//  pub.publish (*cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1;
  cloud1.reset (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg (*input, *cloud1);
//  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//  viewer.showCloud(cloud1,"Simple Cloud Viewer");
  viewer1->addCoordinateSystem (1.0);
  viewer1->removeAllPointClouds();  // 移除当前所有点云
  viewer1->addPointCloud(cloud1, "realsense pcl");
  viewer1->updatePointCloud(cloud1, "realsense pcl");

  viewer1->spinOnce(100);

}
//void receive_image(const sensor_msgs::ImageConstPtr &image_msg){

//    cv_bridge::CvImagePtr cv_ptr;
//    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
//    input_img = cv_ptr->image;
//    cv::imshow(OPENCV_WINDOW, input_img);
//    cv::waitKey(3);
//}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl");
  ros::NodeHandle nh;

  //"/camera/depth/color/points"realsense发布的点云数据
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);
//  ros::Subscriber img_sub = nh.subscribe("/drone/front_camera/color/image_raw", 1,receive_image);//for sim
//  ros::Subscriber img_sub = nh.subscribe("/camera/color/image_raw", 1,receive_image);//for real

//  pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
//  img_height = 480;
//  img_width = 640;
//  input_img = cv::Mat(img_width, img_height, CV_8UC3, cv::Scalar(0, 0, 0));
//  cv::namedWindow(OPENCV_WINDOW);
  // Spin
  ros::spin ();
}
