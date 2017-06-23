
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "a3_help/RequestGoal.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>

namespace enc = sensor_msgs::image_encodings;


struct PoseArrayBuffer
{
    std::mutex buffer_mutex;
    bool send;
    geometry_msgs::PoseArray poses;
    geometry_msgs::Pose vehPose;
    double mapSize;
    double resolution;
};



void CallBackFunc(int event, int x, int y, int flags, void* param)
{

    PoseArrayBuffer* poseArrayBuffer = ((PoseArrayBuffer*)(param)); // 1st cast it back, then deref

     if  ( event == cv::EVENT_LBUTTONDOWN )
     {
          std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;

          // Create one pose and push back onto array
          geometry_msgs::Pose robot_pose;  
          geometry_msgs::Pose pose;  
          poseArrayBuffer->buffer_mutex.lock();
          robot_pose = poseArrayBuffer->vehPose ;
          poseArrayBuffer->buffer_mutex.unlock();
          //!
          //! Convert from Image to Global Coordinates
          //!
          pose.position.y = ( (300/2) - (double)y +  ((double)(robot_pose.position.y) / 0.1)) * 0.1;
	      pose.position.x = (-(300/2) + (double)x +  ((double)(robot_pose.position.x) / 0.1)) * 0.1;
          //BEGIN - THIS IS NOT CORRECT AND ONLY INSERTED HERE SO YOU CAN COMPILE CODE

          //END - THIS IS NOT CORRECT AND ONLY INSERTED HERE SO YOU CAN COMPILE CODE

          poseArrayBuffer->buffer_mutex.lock();
          poseArrayBuffer->poses.poses.push_back(pose);
          poseArrayBuffer->buffer_mutex.unlock();

     }
     else if  ( event == cv::EVENT_RBUTTONDOWN )
     {
          std::cout << "Right button of the mouse is clicked, will send poses soon!" << std::endl;
          poseArrayBuffer->buffer_mutex.lock();
          poseArrayBuffer->send=true;
          poseArrayBuffer->buffer_mutex.unlock();
     }
}


/**
 * This node shows some connections and publishing images
 */


class GazeboRetrieve{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher posearray_pub_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;

    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels
    double mapSize_;//! size of OgMap in pixels

    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex buffer_mutex_;
    };
    ImageDataBuffer imageBuffer;//! And now we have our container

    PoseArrayBuffer poseArrayBuffer;


public:
    GazeboRetrieve(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {
        sub1_ = nh_.subscribe("odom", 1000, &GazeboRetrieve::odomCallback,this);
        image_transport::ImageTransport it(nh);
        sub3_ = it.subscribe("map_image/full", 1, &GazeboRetrieve::imageCallback,this);

        posearray_pub_ = nh_.advertise<geometry_msgs::PoseArray>("path", 100);

        //Below is how to get parameters from command line, on command line they need to be _param:=value
        ros::NodeHandle pn("~");
        pn.param<double>("map_size", mapSize_, 20.0);
        pn.param<double>("resolution", resolution_, 0.1);

        count_ =0;
        cv::namedWindow("view",CV_WINDOW_NORMAL);
        cv::startWindowThread();
        cv::waitKey(5);

        poseArrayBuffer.send=false;// Let's make sending not possible initially
        poseArrayBuffer.mapSize=mapSize_;
        poseArrayBuffer.resolution=resolution_;
        //set the callback function for any mouse event
        cv::setMouseCallback("view", CallBackFunc, &poseArrayBuffer);

    }

    ~GazeboRetrieve()
    {
        cv::destroyWindow("view");
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        geometry_msgs::Pose pose=msg->pose.pose;
        poseArrayBuffer.buffer_mutex.lock();
        poseArrayBuffer.vehPose= pose;
        poseArrayBuffer.buffer_mutex.unlock();

    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        //! Below code pushes the image and time to a deque, to share across threads
        try
        {
          if (enc::isColor(msg->encoding))
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
          else
            cvPtr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        imageBuffer.buffer_mutex_.lock();
        imageBuffer.imageDeq.push_back(cvPtr_->image);

        if((cvPtr_->image.cols !=300) || (cvPtr_->image.rows !=300)){
                ROS_WARN_THROTTLE(60, "The image is not 300 x 300 what has gone wrong!");
        }

        imageBuffer.timeStampDeq.push_back(msg->header.stamp);
        if(imageBuffer.imageDeq.size()>2){
            imageBuffer.imageDeq.pop_front();
            imageBuffer.timeStampDeq.pop_front();
        }
        imageBuffer.buffer_mutex_.unlock();

    }


    void seperateThread() {
       /**
        * The below loop runs until ros is shutdown, to ensure this thread does not remain
        * a zombie thread
        *
        * The loop locks the buffer, checks the size
        * And then pulls items: the pose and timer_t
        * You can think if these could have been combined into one ...
        */

        /// The below gets the current Ros Time
        ros::Time timeImage = ros::Time::now();
        cv::Mat image;


        while (ros::ok()) {

            //! Lock image buffer, take one message from deque and unlock it
            imageBuffer.buffer_mutex_.lock();
            if(imageBuffer.imageDeq.size()>0){
                image = imageBuffer.imageDeq.front();
                timeImage = imageBuffer.timeStampDeq.front();
                imageBuffer.imageDeq.pop_front();
                imageBuffer.timeStampDeq.pop_front();
            }
            imageBuffer.buffer_mutex_.unlock();

            // Update GUI Window
            if(!image.empty()){
                cv::Mat rgbImage;
                cv::cvtColor(image,rgbImage,CV_GRAY2RGB);
                cv::circle(rgbImage, cv::Point((image.rows/2), (image.cols/2)), 3, CV_RGB(0,0,255),-1);


                bool send=false;
                poseArrayBuffer.buffer_mutex.lock();
                send = poseArrayBuffer.send;
                poseArrayBuffer.buffer_mutex.unlock();

                if(send){
                   poseArrayBuffer.buffer_mutex.lock();
                   for(unsigned int i=0;i<poseArrayBuffer.poses.poses.size();i++){
                       geometry_msgs::Pose pose = poseArrayBuffer.poses.poses[i];
                       double xD=0,yD=0;
                       //!
                       //! Convert from Global to Image Coordinates
                       //!

                       //BEGIN - THIS IS NOT CORRECT AND ONLY INSERTED HERE SO YOU CAN COMPILE CODE
                       cv::circle(rgbImage, cv::Point((xD), (yD)), 3, CV_RGB(255,0,0),-1);
                       //END - THIS IS NOT CORRECT AND ONLY INSERTED HERE SO YOU CAN COMPILE CODE

                   }
                   //Send the poses via topic
                   poseArrayBuffer.poses.header.stamp = ros::Time::now();
                   posearray_pub_.publish(poseArrayBuffer.poses);

                   poseArrayBuffer.poses.poses.clear();
                   poseArrayBuffer.send=false;
                   poseArrayBuffer.buffer_mutex.unlock();
                   cv::imshow("view", rgbImage);
                   cv::waitKey(0);//This will hold image until you press any key!
                }
                else
                {
                    cv::imshow("view", rgbImage);
                    cv::waitKey(5);
                }

            }
            else{
                 // This delay slows the loop down for the sake of readability
                std::this_thread::sleep_for (std::chrono::milliseconds(10));
            }
        }
    }

};


int main(int argc, char **argv)
{


  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "gazebo_retrieve");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  /**
   * Let's start seperate thread first, to do that we need to create object
   * and thereafter start the thread on teh function desired
   */
  std::shared_ptr<GazeboRetrieve> gc(new GazeboRetrieve(nh));
  std::thread t(&GazeboRetrieve::seperateThread,gc);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  /**
   * Let's cleanup everything, shutdown ros and join the thread
   */
  ros::shutdown();
  t.join();

  return 0;
}

