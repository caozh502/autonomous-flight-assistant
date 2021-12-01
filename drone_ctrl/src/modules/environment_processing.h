#ifndef SRC_ENVIRONMENT_PROCESSING_H
#define SRC_ENVIRONMENT_PROCESSING_H

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "abstract_module.h"


class environment_processing : public abstract_module{
    ros::NodeHandle nh;
    ros::Publisher env_info_pub;
    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;

public:
    environment_processing();
    ~environment_processing();
    void process_image(const sensor_msgs::ImageConstPtr& img_msg);

};


#endif //SRC_ENVIRONMENT_PROCESSING_H
