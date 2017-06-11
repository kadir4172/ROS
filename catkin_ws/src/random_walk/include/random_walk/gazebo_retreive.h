#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>


class GazeboRetrieve{

private:
    struct DataBuffer
    {
        std::deque<geometry_msgs::Pose> poseDeq;    //! Deque to store odometry message
        std::deque<ros::Time> timeStampDeq;         //! Reque to store time of message
        std::mutex buffer_mutex_;                   //! Mutex used to secure buffer
    };


public:
    GazeboRetrieve(ros::NodeHandle nh);             //! Constructore

    ~GazeboRetrieve();

    void seperateThread();                          //! Thread of execution

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);   //! Odometry Callback
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);//! Laser Calback


private:
    ros::NodeHandle nh_;//! Node Handle
    image_transport::ImageTransport it_;    //! Image transport
    image_transport::Publisher image_pub_;  //! Publisher
    ros::Publisher velocity_pub;
    ros::Subscriber sub1;                   //! Subscriber 1
    ros::Subscriber sub2;                   //! Subscriber 2

    cv::Mat image_;                         //! Storage for OgMap in Mat
    int count_;                             //! A counter to allow executing items on N iterations
    int pixels_;                            //! size of OgMap in pixels

    DataBuffer buffer;                      //! And now we have our container

	double minfrontdistance;
double speed;
double avoidspeed;
double turnrate;

      int randint;
      int randcount;
      int avoidcount;
      bool obs;
        // go into read-think-act loop
        double newturnrate;
double newspeed;
    geometry_msgs::Twist cmdvel;

};


