#include "random_walk/gazebo_retreive.h"

    GazeboRetrieve::GazeboRetrieve(ros::NodeHandle nh)
    : nh_(nh), it_(nh)
    {
        sub1 = nh_.subscribe("odom", 1000, &GazeboRetrieve::odomCallback,this);
        sub2 = nh_.subscribe("base_scan_0", 10, &GazeboRetrieve::laserCallback,this);
        //image_pub_ = it_.advertise("ogmap/image", 1);
        velocity_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1); //TBK_Node publishes RosAria/cmd_vel topic and RosAria node subscribes to it

        ros::NodeHandle pn("~");
        double mapSize;
        double resolution;
        pn.param<double>("map_size", mapSize, 300.0);
        pn.param<double>("resolution", resolution, 0.1);

        pixels_ = (int) mapSize / resolution;
        std::cout << "size:" << mapSize << " resolution:" << resolution <<
                     " pixels:" << pixels_ << std::endl;

        //! Create an OgMap which is a grayscale (mono) image of size pixels
        image_ = cv::Mat::zeros(pixels_,pixels_, CV_8UC1);
        count_ =0;
        cv::waitKey(30);

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


    }

    GazeboRetrieve::~GazeboRetrieve()
    {

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
    }




