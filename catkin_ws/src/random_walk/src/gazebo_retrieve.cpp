#include "random_walk/gazebo_retreive.h"
#include "random_walk/RequestGoal.h"
namespace enc = sensor_msgs::image_encodings;

GazeboRetrieve::GazeboRetrieve(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {
	  //! This subscriber will subscribe to odometry messages to get pose information of robot 
      sub1 = nh_.subscribe("odom", 1000, &GazeboRetrieve::odomCallback,this);
      
      //! This subscriber will subscribe to laser scan messages to get laser sensor data
      sub2 = nh_.subscribe("base_scan_0", 10, &GazeboRetrieve::laserCallback,this);
      sub4 = nh_.subscribe("/path", 10, &GazeboRetrieve::pathCallback,this);

     //subscribe map_image/full to get OGMAP, this will be used to calculate percentage of known configuration space
        image_transport::ImageTransport it(nh);
        sub3_ = it.subscribe("map_image/full", 1, &GazeboRetrieve::imageCallback,this);
        request_goal_client = nh_.serviceClient<a3_help::RequestGoal>("request_goal");

	chatter_pub = nh_.advertise<std_msgs::String>("chatter", 100); //!This publisher will publish goal state after %90 coverage of configuration space

        velocity_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1); //TBK_Node publishes RosAria/cmd_vel topic and RosAria node subscribes to it

        ros::NodeHandle pn("~");
        double mapSize;
        double resolution;
        pn.param<double>("map_size", mapSize, 20.0);
        pn.param<double>("resolution", resolution, 0.1);

        pixels_ = (int) mapSize / resolution;
        std::cout << "size:" << mapSize << " resolution:" << resolution <<
                     " pixels:" << pixels_ << std::endl;

        //! Create an OgMap which is a grayscale (mono) image of size pixels
        image_ = cv::Mat::zeros(pixels_,pixels_, CV_8UC1);
        count_ =0;
        cv::waitKey(30);

    /* initialize random seed: */
    srand (time(NULL));

	minfrontdistance = 0.750;
	speed = 0.200;
	avoidspeed = 0; // -150;
    	turnrate = 40*M_PI/180;

      	randint;
	randcount = 0;
      	avoidcount = 0;
      	obs = false;
        // go into read-think-act loop
        newturnrate=0.0f;
	newspeed=0.0f;

        active_discovery = true; //! Start discovery of map at each initializer
        continue_to_walk = true; //! 

    }

    GazeboRetrieve::~GazeboRetrieve()
    { 

    }

    void GazeboRetrieve::pathCallback(const geometry_msgs::PoseArrayConstPtr& msg){


  ROS_INFO("Path Message is Received");
     for (unsigned int i=0; i < msg->poses.size(); i++)
     {
       std::cout << "Point_" << i << "_on Path // Col:" << msg->poses[i].position.x << " Row:" << msg->poses[i].position.y << std::endl;


     }

     }
    void GazeboRetrieve::imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        double discovered_percentage = 0;

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
        
        int size_y = cvPtr_->image.rows; //!get y size from number of rows
        int size_x = cvPtr_->image.cols; //!get x size from number of columns

	int counter = 0;


        
      if(active_discovery){
      	for (int y = 0; y < size_y; y++){
	  for (int x = 0; x < size_x; x++){

          switch ((int)cvPtr_->image.at<uchar>(x, y))
          {
          case 255:
	  case 0:
            counter++;
            break;

	  case 127: 
          default:
            break;
          }
        }
        }

        discovered_percentage = (counter*100)/9844; //!get discovered percentage of actual map with size of 6.27x15,7[m] and 0.1[m] resolution
	std::cout << "Discovered Percentage: %" << discovered_percentage << std::endl;
        if(discovered_percentage > 80.0){ //! When robot discover the working space more than %90, stop random walk process and publish goal state
          std_msgs::String msg;  
          std::stringstream ss;

	  a3_help::RequestGoal srv;

            continue_to_walk = false;
            srv.request.x = rand() % size_x ;
	    srv.request.y = rand() % size_y;
            if (request_goal_client.call(srv))
            {
              if(srv.response.ack){
                ROS_INFO("Goal Request Confirmed");
		ss << srv.request.x << "," << srv.request.y;
	        msg.data = ss.str();         
                chatter_pub.publish(msg); 
                active_discovery = false; 
              }
              else{
                ROS_INFO("Invalid Goal State");
                std::this_thread::sleep_for (std::chrono::milliseconds(50));
              }
            }
            else
            {
              ROS_INFO("Failed to call service request_goal");
              std::this_thread::sleep_for (std::chrono::milliseconds(50));         
            }
        }
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




    void GazeboRetrieve::odomCallback(const nav_msgs::OdometryConstPtr& msg)
    {
        //Let's get the pose out from odometry message
        // REMEBER: on command line you can view entier msg as
        //rosmsg show nav_msgs/Odometry
        geometry_msgs::Pose pose=msg->pose.pose;
        buffer.buffer_mutex_.lock();
        buffer.poseDeq.push_back(pose);
        buffer.timeStampDeq.push_back(msg->header.stamp);
        buffer.buffer_mutex_.unlock();

    }



    void GazeboRetrieve::laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
    {
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

        obs = false;
        for (unsigned int i = 0; i < msg->ranges.size(); i++){
              if(msg->ranges.at(i) < minfrontdistance){
                obs = true;
            break;
            }
        }

        if(obs || avoidcount )
        {
            newspeed = avoidspeed;
            // once we start avoiding, continue avoiding for 2 seconds 
            // (we run at about 10Hz, so 20 loop iterations is about 2 sec) 
            if(!avoidcount)
            {
              avoidcount = 15;
              randcount = 0;
                if(msg->ranges.at(msg->ranges.size()-1) <  msg->ranges.at(0))
                  newturnrate = -turnrate;
                else
                  newturnrate = turnrate;
            }
            avoidcount--;
          }
          else
          {
            avoidcount = 0;
            newspeed = speed;
            // update turnrate every 3 seconds 
            if(!randcount)
            {
              // make random int tween -20 and 20 
              //randint = (1+(int)(40.0*rand()/(RAND_MAX+1.0))) - 20;
              randint = rand() % 41 - 20;
              newturnrate = randint*M_PI/180.0;
              randcount = 20;
            }
            randcount--;
          }
    }


    void GazeboRetrieve::seperateThread() {
       /**
        * The thread runs until ros is shutdown, to ensure this thread does not remain
        * a zombie thread
        *
        * The loop gets the velocity and turnrate computed from other thread
        * and then feeds the velocity controller of the robot with the speed/turnrate
		* Sleeps for 20ms thereafter
        */

        double yaw,x,y;
        /// The below gets the current Ros Time
        ros::Time timeOdom = ros::Time::now();;
        while (ros::ok()) {
          if(continue_to_walk){
            int deqSz =-1;

            buffer.buffer_mutex_.lock();
            deqSz = buffer.poseDeq.size();
            if (deqSz > 0) {
                geometry_msgs::Pose pose=buffer.poseDeq.front();
                yaw = tf::getYaw(pose.orientation);
                x = pose.position.x;
                y = pose.position.y;
                timeOdom = buffer.timeStampDeq.front();
                buffer.poseDeq.pop_front();
                buffer.timeStampDeq.pop_front();
            }
            buffer.buffer_mutex_.unlock();


            // write commands to robot
            cmdvel.linear.x = newspeed;
            cmdvel.angular.z = newturnrate;
			// printf("sending...[%f, %f]\n", cmdvel.linear.x, cmdvel.angular.z);


	    velocity_pub.publish(cmdvel);
            std::this_thread::sleep_for (std::chrono::milliseconds(10));
          }
          else{
            cmdvel.linear.x = 0;
            cmdvel.angular.z = 0;
            velocity_pub.publish(cmdvel);
            break;
          }
            


        }
    }




