//http://wiki.ros.org/image_transport/Tutorials/PublishingImages

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");

    ROS_WARN(" RUN AS: rosrun opencv_example image_publisher _image:=~/lena.jpg");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    ros::NodeHandle pn("~");
    std::string imageName;
    pn.param<std::string>("image", imageName, "lena.jpg");

    cv::Mat image = cv::imread(imageName.c_str(), CV_LOAD_IMAGE_COLOR);
    cv::waitKey(30);

    /* initialize random seed: */
    srand (time(NULL));
    /* circle radious is 10 */
    int circle_radius=10;

    ros::Rate loop_rate(5);

    while (nh.ok()) {
        //AA: Why does the below statement keep overwritting image with circles?
        //cv::Mat tmp(image);
        cv::Mat tmp = image.clone();

        /* generate secret number between
        * 1 and min (rows /cols) - circle_radius
        */
        int x = rand() % image.cols + 1;
        int y = rand() % image.rows + 1;

        //draw filled circle
        cv::circle(tmp, cv::Point(x, y), circle_radius, CV_RGB(255,0,0),-1);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tmp).toImageMsg();

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
