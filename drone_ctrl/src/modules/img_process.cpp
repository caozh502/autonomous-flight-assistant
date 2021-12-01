#include "img_process.h"
static const std::string OPENCV_WINDOW = "Image window";

img_process::img_process()
    :nh(),it(nh)
{
    img_sub = it.subscribe("/drone/front_camera/color/image_raw", 1,
                             &img_process::receive_image, this);//for sim
//    img_sub = nh.subscribe("/camera/color/image_raw", 1,
//                             &img_process::receive_image, this);//for real
    this->img_height = 480;
    this->img_width = 640;
    this->input_img =
            cv::Mat(this->img_width, this->img_height, CV_8UC3, cv::Scalar(0, 0, 0));

    cv::namedWindow(OPENCV_WINDOW);
}

img_process::~img_process(){}

void img_process::receive_image(const sensor_msgs::ImageConstPtr &image_msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    this->input_img = cv_ptr->image;
    std::cout<<1<<std::endl;
    cv::imshow(OPENCV_WINDOW, this->input_img);
    std::cout<<3<<std::endl;
    cv::waitKey(30);

    std::cout<<2<<std::endl;
}
void img_process::destroywindow(){
    cv::destroyWindow(OPENCV_WINDOW);
}
