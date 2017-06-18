#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

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
	//! This struct will hold a deque of Pose messages and time stamps of these messages and a mutex to share data safely
    struct DataBuffer
    {
      //! Deque to store odometry message
      std::deque<geometry_msgs::Pose> poseDeq;
      
      //! Deque to store time of message
      std::deque<ros::Time> timeStampDeq;
      
      //! Mutex used to secure buffer
      std::mutex buffer_mutex_;                   
    };
    
    //! This struct will hold a deque of cv::Mat messages and time stamps of these messages and a mutex to share data safely
    struct ImageDataBuffer
    {
    	 //! Deque to store image
        std::deque<cv::Mat> imageDeq;
        
        //! Deque to store time of image
        std::deque<ros::Time> timeStampDeq;
        
        //! Mutex used to secure buffer
        std::mutex buffer_mutex_;		  
    };


public:
    //! Constructor
    GazeboRetrieve(ros::NodeHandle nh);           

    //! Destructor
    ~GazeboRetrieve();

    //! Thread of execution
    void seperateThread();

    //!Image Callback
    void imageCallback(const sensor_msgs::ImageConstPtr&);		

private:
    //! Odometry Callback
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);  
    
    //! Laser Calback
    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);
    
    //! Path Calback
    void pathCallback(const geometry_msgs::PoseArrayConstPtr& msg);

private:
    //! Node Handle
    ros::NodeHandle nh_;
    
    //! Image transport
    image_transport::ImageTransport it_;
    
    //! Image transport
    image_transport::Publisher image_pub_;
    
    //! Velocity Command Publisher
    ros::Publisher velocity_pub;  
    
    //! Goal State Publisher
    ros::Publisher chatter_pub;
    
    //! Subscriber 1
    ros::Subscriber sub1;
    
    //! Subscriber 2
    ros::Subscriber sub2;
    
    //! Subscriber 3
    image_transport::Subscriber sub3_;
    
    //! Subscriber 4
    ros::Subscriber sub4;
    
    //! Request Goal Service Client
    ros::ServiceClient request_goal_client;
    
    //! Image pointer to get OGMAP
    cv_bridge::CvImagePtr cvPtr_;	        

    //! Storage for OgMap in Mat
    cv::Mat image_;             
    
    //! A counter to allow executing items on N iterations
    int count_;
    
    //! size of OgMap in pixels
    int pixels_;                            

    
    //! Data buffer to store position data from odometer
    DataBuffer buffer;                      
    
    //! Image buffer to store image messages
    ImageDataBuffer imageBuffer;           

    //! Minimum front distance to avoid collision
    double minfrontdistance;
    
    //! Speed variable for random walk
    double speed;
    
    //! Avoid speed variable for random walk
    double avoidspeed;
    
    //! Turn rate variable for random walk
    double turnrate;

    //! Flag to request goal position after certain threshold of discovery percentage
    bool active_discovery;
    
    //! Flag to stop random walk after certain threshold of discovery percentage
    bool continue_to_walk;
    
    //! Random variable for random walk
    int randint;
    
    //! Random variable for random walk
    int randcount;
    
    //! Counter variable for random walk
    int avoidcount;
      
    //! Counter variable for random walk
    bool obs;
        
    //! Turn rate variable for random walk
    double newturnrate;

    //! Speed variable for random walk
    double newspeed;
    
    //! Cmdvel message to publish command
    geometry_msgs::Twist cmdvel;

};


