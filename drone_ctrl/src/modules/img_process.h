#ifndef IMG_PROCESS_H
#define IMG_PROCESS_H
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "abstract_module.h"

class img_process: public abstract_module{
ros::NodeHandle nh;
//ros::Subscriber img_sub;
image_transport::ImageTransport it;
image_transport::Subscriber img_sub;


cv::Mat input_img;
int img_width;
int img_height;

public:
    img_process();
    ~img_process();
    void destroywindow();
private:
    void receive_image(const sensor_msgs::ImageConstPtr& img_msg);
};

#endif // IMG_PROCESS_H
