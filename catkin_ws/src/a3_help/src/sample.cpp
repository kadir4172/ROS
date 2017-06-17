
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include <opencv2/imgproc/imgproc.hpp>
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

/**
 * This node shows some connections and publishing images
 */


class GazeboRetrieve{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Subscriber sub4_;                       //! This subscriber will get goal state when map discovery is finished
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceServer service_;


    int count_;//! A counter to allow executing items on N iterations
    double resolution_;//! size of OgMap in pixels

    struct DataBuffer
    {
        std::deque<geometry_msgs::Pose> poseDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex buffer_mutex_;
    };
    DataBuffer buffer;//! And now we have our container

    struct ImageDataBuffer
    {
        std::deque<cv::Mat> imageDeq;
        std::deque<ros::Time> timeStampDeq;
        std::mutex buffer_mutex_;
    };
    ImageDataBuffer imageBuffer;//! And now we have our container


public:
    GazeboRetrieve(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {
        sub1_ = nh_.subscribe("odom", 1000, &GazeboRetrieve::odomCallback,this);
        sub2_ = nh_.subscribe("base_scan_0", 10, &GazeboRetrieve::laserCallback,this);
        sub4_ = nh_.subscribe("chatter", 100, &GazeboRetrieve::chatterCallback, this);
        image_transport::ImageTransport it(nh);
        sub3_ = it.subscribe("map_image/full", 1, &GazeboRetrieve::imageCallback,this);

        //Publishing an image ... just to show how
        image_pub_ = it_.advertise("test/image", 1);

        //Allowing an incoming service
        service_ = nh_.advertiseService("request_goal", &GazeboRetrieve::requestGoal,this);

        //Below is how to get parameters from command line, on command line they need to be _param:=value
        ros::NodeHandle pn("~");
        double mapSize;
        double resolution;
        pn.param<double>("map_size", mapSize, 300.0);
        pn.param<double>("resolution", resolution_, 0.1);

        count_ =0;
        //cv::namedWindow("view",CV_WINDOW_NORMAL);
        //cv::startWindowThread();
        //cv::waitKey(5);

    }

    ~GazeboRetrieve()
    {
        //cv::destroyWindow("view");
    }

    void chatterCallback(const std_msgs::String::ConstPtr& msg)
    {
     int counter = 0;
     int goal_col;
     int goal_row;
     std::stringstream s;
     std::string token;
     s << msg->data.c_str();
     ROS_INFO("I heard: [%s]", msg->data.c_str()); 
     while(std::getline(s, token, ',')) {
       std::cout << token << '\n';
       if(counter == 0){
         goal_col = std::stoi(token);   
       }
       else{
         goal_row = std::stoi(token);
       }
       counter++;
     }
//TODO call shortest path function here
    }


    //This function will be used to check whether a pixel in OGMAP is in free configuration space by augmenting it Minkowski sums
    bool isConfigurationFree(int local_y, int local_x){
      cv::Mat image;
      //! Lock image buffer, take one message from deque and unlock it
      imageBuffer.buffer_mutex_.lock();
      if(imageBuffer.imageDeq.size()>0){
        image = imageBuffer.imageDeq.front();
        imageBuffer.buffer_mutex_.unlock();
        
        if((local_y<=0) || (local_y>=image.rows-1) || (local_x<=0) || (local_x>=image.cols-1)){
          ROS_INFO("Point is out of boundaries");
          return false; 
        }
        //! Check up, down, left, right cells to calculate local configuration space with Minkowski sums
        if(((int)image.at<uchar>(local_y  , local_x  ) == 255) && 
	   ((int)image.at<uchar>(local_y+1, local_x  ) == 255) &&
	   ((int)image.at<uchar>(local_y-1, local_x  ) == 255) &&
	   ((int)image.at<uchar>(local_y  , local_x+1) == 255) &&
	   ((int)image.at<uchar>(local_y  , local_x-1) == 255)){

           return true;
	}
        else{
           return false;
        }
      }
      else{
        ROS_INFO("No Available OGMAP to check");
        imageBuffer.buffer_mutex_.unlock();
      }
    }



    bool requestGoal(a3_help::RequestGoal::Request  &req,
             a3_help::RequestGoal::Response &res)
    {
      ROS_INFO("request: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
      if(isConfigurationFree((int)req.y, (int)req.x)){
        res.ack = true;
        ROS_INFO("sending back response: [%d]", res.ack);
        return true;
      }
      else{
        res.ack = false;
        ROS_INFO("sending back response: [%d]", res.ack);
        return true;
      }
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        geometry_msgs::Pose pose=msg->pose.pose;
        buffer.buffer_mutex_.lock();
        buffer.poseDeq.push_back(pose);
        buffer.timeStampDeq.push_back(msg->header.stamp);
        if(buffer.poseDeq.size()>2){
            buffer.poseDeq.pop_front();
            buffer.timeStampDeq.pop_front();
        }
        buffer.buffer_mutex_.unlock();

        std::cout << "Pose x: " << pose.position.x << "  Pose y: "<< pose.position.y << std::endl;

    }

    bool global2image(double global_x, double global_y, int* local_x, int* local_y){
            cv::Mat image;
            imageBuffer.buffer_mutex_.lock();
            if(imageBuffer.imageDeq.size()>0){
                image = imageBuffer.imageDeq.front();
                imageBuffer.buffer_mutex_.unlock();
            }
            else{
                ROS_INFO("global2image: Unable to convert, no image found on buffer");
                imageBuffer.buffer_mutex_.unlock();
                return false;
            }

 

            buffer.buffer_mutex_.lock();
            if (buffer.poseDeq.size() > 0) {
                geometry_msgs::Pose pose=buffer.poseDeq.front();
		*local_y = (int)((double)image.rows/2 + (double)(pose.position.y - global_y) / resolution_);
		*local_x = (int)((double)image.cols/2 - (double)(pose.position.x - global_x) / resolution_);
		buffer.buffer_mutex_.unlock();
                return true;                  
                
            }
            else{
                ROS_INFO("global2image: Unable to convert, no position info found on buffer");
                buffer.buffer_mutex_.unlock();
                return false;
            }
    }

    bool image2global(double* global_x, double* global_y, int local_x, int local_y){
            cv::Mat image;
            imageBuffer.buffer_mutex_.lock();
            if(imageBuffer.imageDeq.size()>0){
                image = imageBuffer.imageDeq.front();
                imageBuffer.buffer_mutex_.unlock();
            }
            else{
                ROS_INFO("image2global: Unable to convert, no image found on buffer");
                imageBuffer.buffer_mutex_.unlock();
                return false;
            }

 

            buffer.buffer_mutex_.lock();
            if (buffer.poseDeq.size() > 0) {
                geometry_msgs::Pose pose=buffer.poseDeq.front();
		*global_y = ( (image.rows/2) - (double)local_y +  ((double)(pose.position.y) / resolution_)) * resolution_;
		*global_x = (-(image.cols/2) + (double)local_x +  ((double)(pose.position.x) / resolution_)) * resolution_;
		buffer.buffer_mutex_.unlock();
                return true;                  
                
            }
            else{
                ROS_INFO("image2global: Unable to convert, no position info found on buffer");
                buffer.buffer_mutex_.unlock();
                return false;
            }
    }

    void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
        //! Below sample cicrulates through the scan and finds closest point to robot
        double closest_point=msg->range_max;
        double angle=0;
        double x,y;
        for (unsigned int idx=0 ; idx < msg->ranges.size() ; idx++){
            if(msg->ranges.at(idx)<closest_point){
                closest_point=msg->ranges.at(idx);
                angle=msg->angle_min+(idx*msg->angle_increment);
            }
        }
        ros::Time timeLaser = msg->header.stamp;
        x = closest_point * cos(angle);
        y = closest_point * sin(angle);
        //std::cout << timeLaser << " L: [d,theta,x,y]=[" << closest_point << "," << angle << "," << x << "," << y << "]" << std::endl;
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

        double yaw,x,y;
        /// The below gets the current Ros Time
        ros::Time timeOdom = ros::Time::now();;
        ros::Time timeImage = ros::Time::now();;
        cv::Mat image;
	cv::Mat tmp ;		

	geometry_msgs::Pose pose;

        while (ros::ok()) {
            int deqSz =-1;

            buffer.buffer_mutex_.lock();
            deqSz = buffer.poseDeq.size();
            if (deqSz > 0) {
                pose=buffer.poseDeq.front();
                yaw = tf::getYaw(pose.orientation);
                x = pose.position.x;
                y = pose.position.y;
                timeOdom = buffer.timeStampDeq.front();
                //buffer.poseDeq.pop_front();
                //buffer.timeStampDeq.pop_front();
            }
            buffer.buffer_mutex_.unlock();

            //! Lock image buffer, take one message from deque and unlock it
            imageBuffer.buffer_mutex_.lock();
            if(imageBuffer.imageDeq.size()>0){
                image = imageBuffer.imageDeq.front();
                timeImage = imageBuffer.timeStampDeq.front();
                //imageBuffer.imageDeq.pop_front();
                //imageBuffer.timeStampDeq.pop_front();
            }
            imageBuffer.buffer_mutex_.unlock();

            //Let's do something with the incoming ogMap now;
            //on every 100 iterations
            if (count_>100){
               count_=0;


               std::cout << "rows"  << image.rows << std::endl;
               std::cout << "cols"  << image.cols << std::endl;
               int local_x ;
               int local_y ;
               double global_x;
               double global_y;
    
               global2image(pose.position.x, pose.position.y, &local_x, &local_y);
               image2global(&global_x, &global_y, local_x, local_y);

               bool result = isConfigurationFree(local_y, local_x);
               std::cout << "local_y:"  << local_y  << " local_x:"  << local_x  << " is:" << result << std::endl;   
               std::cout << "global_y:" << global_y << " global_x:" << global_x << " is:" << result << std::endl;   

 	       cv::cvtColor(image,tmp,CV_GRAY2RGB);
               cv::circle(tmp, cv::Point(local_x, local_y), 1, CV_RGB(0,0,255),-1);
               sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", tmp).toImageMsg();
               image_pub_.publish(msg);

            }
            else{
                count_++;
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

