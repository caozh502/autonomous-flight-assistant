#include "environment_processing.h"

static const std::string OPENCV_WINDOW = "Image window";

environment_processing::environment_processing()
        : nh(), it(nh)
{
    env_info_pub = nh.advertise<std_msgs::String>("/drone/env_info", 1000);
    image_sub = it.subscribe("/drone/front_camera/color/image_raw", 1,
                             &environment_processing::process_image, this);
    cv::namedWindow(OPENCV_WINDOW);
}

environment_processing::~environment_processing()
{
    cv::destroyWindow(OPENCV_WINDOW);
}

void environment_processing::process_image(const sensor_msgs::ImageConstPtr& img_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat greyMat;
    cv::cvtColor(cv_ptr->image, greyMat, CV_BGR2GRAY); //convert image to grayscale
    cv::Scalar mean = cv::mean(greyMat);
    ROS_INFO_STREAM("Mean Pixel intensity" << mean);

    std_msgs::String env_info_msg;

    if(mean[0] < 130){  // Example: if average Intensity is too low -> go up, if too high -> do down
        env_info_msg.data = "dark";
        ROS_INFO_STREAM("It's dark here");
    }
    else{
        env_info_msg.data = "bright";
        ROS_INFO_STREAM("It's bright here");
    }
    env_info_pub.publish(env_info_msg);

    cv::circle(greyMat, cv::Point(50, 50), 10, CV_RGB(255,0,0)); // draw example circle
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, greyMat);
    cv::waitKey(3);
}
