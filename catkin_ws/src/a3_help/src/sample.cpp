
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
#include <geometry_msgs/PoseArray.h>

#include "a3_help/RequestGoal.h"

#include <sstream>
#include <iostream>
#include <string>

#include <thread>
#include <chrono>
#include <deque>
#include <mutex>
#include <random>
#include <stdio.h>
#include <limits.h>
#include <math.h>
#include <vector>
#include <tuple>
#include <stdlib.h>
#include <algorithm>

namespace enc = sensor_msgs::image_encodings;

/**
 * This node shows some connections and publishing images
 */



#define BUFFER_SIZE 10000
cv::Mat image;
cv::Mat tmp ;	
std::vector< std::tuple<int,int> > configuration_point_list;
//This array will hold distance of shortest paths to node j from source
int distance[BUFFER_SIZE];


//This array stores info about whether node i is used currently or not
bool is_point_used[BUFFER_SIZE];

// This parent array stores shortest paths in tree
int parent[BUFFER_SIZE];


class GazeboRetrieve{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub1_;
    image_transport::Publisher image_pub2_;
    ros::Subscriber sub1_;
    ros::Subscriber sub2_;
    ros::Subscriber sub4_;                       //! This subscriber will get goal state when map discovery is finished
    image_transport::Subscriber sub3_;
    cv_bridge::CvImagePtr cvPtr_;
    ros::ServiceServer service_;
    ros::Publisher path_publisher;
    geometry_msgs::PoseArray path_to_publish;
    geometry_msgs::Pose      node_on_path;

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
        path_publisher = nh_.advertise<geometry_msgs::PoseArray>("/path",1);

        //Publishing an image ... just to show how
        image_pub1_ = it_.advertise("roadmap/first_query", 1);
        image_pub2_ = it_.advertise("roadmap/second_query", 1);

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
     int min_col;
     int min_row;
     int max_col;
     int max_row;
     int start_col;
     int start_row;
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
     while(!global2image((-6.27/2), (15.7/2), &min_col, &min_row)){}
     while(!global2image((6.27/2), (-15.7/2), &max_col, &max_row)){}
    
     start_row = (int)image.rows/2;
     start_col = (int)image.cols/2;


     std::cout << "Start Row: " << start_row << std::endl;
     std::cout << "Start Col: " << start_col << std::endl;
     std::cout << "Goal  Row: " << goal_row  << std::endl;
     std::cout << "Goal  Col: " << goal_col  << std::endl;
     std::cout << "Min   Row: " << min_row   << std::endl;
     std::cout << "Min   Col: " << min_col   << std::endl;
     std::cout << "Max   Row: " << max_row   << std::endl;
     std::cout << "Max   Col: " << max_col   << std::endl;



  int number_of_configurations = 12;
  while(!detect_regions(min_col, min_row, max_col, max_row, 2,2, number_of_configurations-2, start_row, start_col, goal_row, goal_col)){
    number_of_configurations+= 10;
    if(number_of_configurations > 100){
      ROS_INFO("No Shortest Path is found even with more configuration points, please restart random_walk component and this component");
      return;
    }
  }
  sensor_msgs::ImagePtr msg_to_dump_first_roadmap = cv_bridge::CvImage(std_msgs::Header(), "rgb8", tmp).toImageMsg();
  image_pub1_.publish(msg_to_dump_first_roadmap);

           goal_col = rand() % (int)(image.cols-1) + 1;
	   goal_row = rand() % (int)(image.rows-1) + 1;
           while(!isConfigurationFree(goal_row, goal_col)){             
           goal_col = rand() % (int)(image.cols-1) + 1;
	   goal_row = rand() % (int)(image.rows-1) + 1;  
          }

ROS_INFO("New Random Goal Point will be added to current roadmap");
std::cout << "Goal_Row: " << goal_row << " Goal_Column: " << goal_col << std::endl; 

    if(add_start_goal_and_calculate_shortest_path(configuration_point_list.size(),  start_row,  start_col,  goal_row,  goal_col)){
     //return;  
    }
    else{
      ROS_INFO("detect_regions: No Path Found to new goal point, its needed to generate a new roadmap with more configuration points"); 
      return;       
    }

 sensor_msgs::ImagePtr msg_to_dump_second_roadmap = cv_bridge::CvImage(std_msgs::Header(), "rgb8", tmp).toImageMsg();
  image_pub2_.publish(msg_to_dump_second_roadmap);


    }


    //This function will be used to check whether a pixel in OGMAP is in free configuration space by augmenting it Minkowski sums
    bool isConfigurationFree(int local_y, int local_x){

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
        return false;
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

        //std::cout << "Pose x: " << pose.position.x << "  Pose y: "<< pose.position.y << std::endl;

    }

    bool global2image(double global_x, double global_y, int* local_x, int* local_y){
            cv::Mat image_tmp;
            imageBuffer.buffer_mutex_.lock();
            if(imageBuffer.imageDeq.size()>0){
                image_tmp = imageBuffer.imageDeq.front();
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
		*local_y = (int)((double)image_tmp.rows/2 + (double)(pose.position.y - global_y) / resolution_);
		*local_x = (int)((double)image_tmp.cols/2 - (double)(pose.position.x - global_x) / resolution_);
		buffer.buffer_mutex_.unlock();
                std::cout << "incoming request global_x" << global_x << std::endl;
                std::cout << "incoming request global_y" << global_y << std::endl;
                std::cout << "calculated position local_x" << *local_x << std::endl;
                std::cout << "calculated position local_y" << *local_y << std::endl;
                return true;                  
                
            }
            else{
                ROS_INFO("global2image: Unable to convert, no position info found on buffer");
                buffer.buffer_mutex_.unlock();
                return false;
            }
    }

    bool image2global(double* global_x, double* global_y, int local_x, int local_y){
            cv::Mat image_tmp;
            imageBuffer.buffer_mutex_.lock();
            if(imageBuffer.imageDeq.size()>0){
                image_tmp = imageBuffer.imageDeq.front();
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
		*global_y = ( (image_tmp.rows/2) - (double)local_y +  ((double)(pose.position.y) / resolution_)) * resolution_;
		*global_x = (-(image_tmp.cols/2) + (double)local_x +  ((double)(pose.position.x) / resolution_)) * resolution_;
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
        * Loop is reserved for future usage
        */


        while (ros::ok()) {      
                std::this_thread::sleep_for (std::chrono::milliseconds(10));
            
        }
    }







bool check_line_segment(int start_row, int start_col, int end_row, int end_col){

    cv::LineIterator it(image, cv::Point(start_col, start_row), cv::Point(end_col, end_row), 8);
	for(int i = 0; i < it.count; i++, ++it)
	{
           //check point is free 
	   if(!isConfigurationFree(it.pos().y,it.pos().x)){
		   return false;
	   }
	}

    cv::line  (tmp, cv::Point(start_col, start_row), cv::Point(end_col, end_row), CV_RGB(255,0,0), 1);
	return true;
}









bool detect_regions(int min_col, int min_row, int max_col, int max_row, int number_of_regions_in_col, int number_of_regions_in_row, int total_number_of_configurations, int start_row, int start_col, int goal_row, int goal_col){
	double row_length = (max_row - min_row) / number_of_regions_in_row;
	double col_length   = (max_col    - min_col) / number_of_regions_in_col;
    std::cout << "row_length: " << row_length << std::endl;
    std::cout << "col_length: " << col_length << std::endl;
	int total_occupied_cells = 0;
	int row_region;
	int col_region;

	std::vector< std::vector<int> > occupied_pixel_counter(number_of_regions_in_row, std::vector<int>(number_of_regions_in_col));
	std::vector< std::vector<int> > number_of_configurations(number_of_regions_in_row, std::vector<int>(number_of_regions_in_col));
    //std::cout << "buraya geldi" <<  occupied_pixel_counter[7][20]  << std::endl;




    for(int i = min_row; i<max_row; i++){
    	for(int j = min_col; j<max_col; j++){
            row_region = std::min((int)(floor(((double)i - min_row) / row_length)), (number_of_regions_in_row-1));
            col_region = std::min((int)(floor(((double)j - min_col) / col_length)), (number_of_regions_in_col-1));
            //std::cout << "i: " << i << std::endl;
            //std::cout << "j: " << j << std::endl;
            //std::cout << "row_region: " << row_region << std::endl;
            //std::cout << "col_region: " << col_region << std::endl;
            if(!isConfigurationFree(i,j)){ //if that pixel is not free
            	occupied_pixel_counter[row_region][col_region] ++;
            	total_occupied_cells ++;
            }
    	}
    }

    std::cout << "total_occupied_cells" << total_occupied_cells<<  std::endl;
    int total_configuration_points = 0;
    for(int k = 0; k<number_of_regions_in_row; k++){
    	for(int l = 0; l<number_of_regions_in_col; l++){
    		std::cout << "region counter:" << occupied_pixel_counter[k][l] << std::endl;
    		number_of_configurations[k][l] =  (int)(floor((double)(total_number_of_configurations * occupied_pixel_counter[k][l]) / total_occupied_cells));
    		total_configuration_points += number_of_configurations[k][l];
    		std::cout << "number of points to be assigned to this region:" << number_of_configurations[k][l] << std::endl;
    	}
    }
    std::cout << "Number of Additional Configurations to be added" << total_number_of_configurations - total_configuration_points << std::endl;
    number_of_configurations[number_of_regions_in_row-1][number_of_regions_in_col-1] += total_number_of_configurations - total_configuration_points;
    int configuration_point_row;
    int configuration_point_col;
    int points_in_region = 0;

    configuration_point_list.clear();
    for(int k = 0; k<number_of_regions_in_row; k++){
    	for(int l = 0; l<number_of_regions_in_col; l++){
    		points_in_region = 0;
            int region_min_row = min_row + k          *   (int)row_length;
            int region_max_row = min_row + (k+1) *  (int)row_length;
            int region_min_col  = min_col + l          *   (int)col_length;
            int region_max_col = min_col + (l+1) *  (int)col_length;
            std::cout << "region:" << k << " ," << l << std::endl;
            std::cout << "region_min_row: " << region_min_row << std::endl;
            std::cout << "region_max_row: " << region_max_row << std::endl;
            std::cout << "region_min_col: " << region_min_col << std::endl;
            std::cout << "region_max_col: " << region_max_col << std::endl;

            while(points_in_region < number_of_configurations[k][l] ){
            	configuration_point_row = region_min_row + rand() % (int)row_length;
            	configuration_point_col = region_min_col + rand() % (int)col_length;
            	//check whether this point is free
            	if(isConfigurationFree(configuration_point_row, configuration_point_col)){
            		points_in_region ++ ;
            		configuration_point_list.push_back( std::tuple<int, int>(configuration_point_row,configuration_point_col) );
            		std::cout << "Node: " << configuration_point_list.size() << " " << configuration_point_row << "," << configuration_point_col << std::endl;
            	}
            }
    	}
    }

    if(add_start_goal_and_calculate_shortest_path(configuration_point_list.size(),  start_row,  start_col,  goal_row,  goal_col)){
     return true;  
    }
    else{
      ROS_INFO("detect_regions: No Path Found, will try with more configuration points"); 
      return false;       
    }


}




//This function returns minimum distance to uncovered vertices
int find_min_distance(int* distance, bool* is_point_used, int size_of_graph)
{
    //determine  minimum value as maximum possible value
    int min = INT_MAX, min_index;

    for (int v = 0; v < size_of_graph; v++)
        if (is_point_used[v] == false && distance[v] <= min)
            min = distance[v], min_index = v;

    return min_index;
}



//This recursive function sweeps parent list up to source itself
void printPath(int parent[], int j)
{
    // Base Case : If j is source
    if (parent[j]==-1)
        return;

    printPath(parent, parent[j]);

    node_on_path.position.x = std::get<1>(configuration_point_list.at(j));
    node_on_path.position.y = std::get<0>(configuration_point_list.at(j));
    node_on_path.position.z = 0;
    node_on_path.orientation.x = 0;
    node_on_path.orientation.y = 0;
    node_on_path.orientation.z = 0;
    node_on_path.orientation.w = 0;
    path_to_publish.poses.push_back(node_on_path);
    cv::line  (tmp, cv::Point(std::get<1>(configuration_point_list.at(j)), std::get<0>(configuration_point_list.at(j))), cv::Point(std::get<1>(configuration_point_list.at(parent[j])), std::get<0>(configuration_point_list.at(parent[j]))), CV_RGB(0,0,255), 2);
    //printf("  %d \n", parent[j]);
    printf("  %d ", j);
}




// A utility function to print the constructed distance
// array
int printSolution(int dist[], int n, int parent[], int size_of_graph)
{
    int src = 0;
    printf("Vertex\t  Distance\tPath");
    for (int i = 1; i < size_of_graph; i++)
    {
        printf("\n%d -> %d \t\t %d\t\t%d ", src, i, dist[i], src);
        printPath(parent, i);
    }
    return 0;
}

//Dijkstra algorithm to find shortest path, given an adjacency graph
void Shortest_Path(int**graph, int src, int size_of_graph)
{


    //determine  initial minimum value as maximum possible and is_point_used to false
    for (int i = 0; i < size_of_graph; i++)
    {
        parent[0] = -1;
        distance[i] = INT_MAX;
        is_point_used[i] = false;
    }

    // Distance of source node to itself is always zero
    distance[src] = 0;

    //For all nodes in adjacency graph, find the shortest path from source
    for (int count = 0; count < size_of_graph-1; count++)
    {
       //Find minimum distance between unused nodes
        int u = find_min_distance(distance, is_point_used, size_of_graph);

        //Change flag of this minimum distance node to 'used'
        is_point_used[u] = true;

        //Update distances to this node
        for (int v = 0; v < size_of_graph; v++)

        	// Update distance[v] only if its not used
            if (!is_point_used[v] && graph[u][v] &&
                distance[u] + graph[u][v] < distance[v])
            {
                parent[v]  = u; //Update parent list of v
                distance[v] = distance[u] + graph[u][v]; //Update distance array of v
            }
    }

    // print the constructed distance array
    //printSolution(distance, NUMBER_OF_NODES, parent);
}




void test_ordered_multimap(int* length_array, std::multimap<int,int> first, int size_of_graph){

      int counter = 1;
	  for (std::multimap<int,int>::iterator it=first.begin(); it!=first.end(); ++it){
		  if(counter <= 6){
			  length_array[(*it).second - 1] = (*it).first;
		  }
		  else{
			  length_array[(*it).second - 1] = 0;
		  }
		  counter ++;
	  }

	  for(int i = 0; i < size_of_graph; i++){
		  std::cout << length_array[i] << ";";
	  }
	  std::cout << std::endl;

}




bool add_start_goal_and_calculate_shortest_path(int total_number_of_configurations, int start_row, int start_col, int goal_row, int goal_col){

            imageBuffer.buffer_mutex_.lock();
            while(imageBuffer.imageDeq.size()<=0){
                ROS_INFO("detect_regions: Unable to proceed, no image found on buffer"); 
                imageBuffer.buffer_mutex_.unlock(); 
                return false;              
            }
            image = imageBuffer.imageDeq.front();
            cv::cvtColor(image,tmp,CV_GRAY2RGB);
            imageBuffer.buffer_mutex_.unlock(); 



    std::vector< std::tuple<int,int> >::iterator it = configuration_point_list.begin();
    configuration_point_list.insert ( it , std::tuple<int, int>(goal_row,goal_col) );
    it = configuration_point_list.begin();
    configuration_point_list.insert ( it , std::tuple<int, int>(start_row,start_col) );
    cv::putText(tmp, "Start", cv::Point(start_col, start_row), 1, 0.8, cv::Scalar(20,20,20), 1, 8);
    cv::putText(tmp, "Goal" , cv::Point(goal_col , goal_row) , 1, 0.8, cv::Scalar(20,20,20), 1, 8);

	int length_array[ total_number_of_configurations +2];
	int length;
	int **graph;
	graph = new int *[total_number_of_configurations +2];
	for(int i = 0; i <(total_number_of_configurations+2); i++){
	    graph[i] = new int[total_number_of_configurations+2];
	}

	std::multimap<int,int> point_list;
   for (std::vector< std::tuple<int,int> >::iterator it1 = configuration_point_list.begin() ; it1 != configuration_point_list.end(); ++it1){
	   std::cout << "Distances to node" << std::distance(configuration_point_list.begin(), it1) << ": "<< std::get<0>(*it1) << " , "<< std::get<1>(*it1) << std::endl;
	   cv::circle(tmp,cv::Point(std::get<1>(*it1),std::get<0>(*it1)), 3, CV_RGB(0,255,0),-1);
	   
   //std::this_thread::sleep_for (std::chrono::milliseconds(500));
     for (std::vector< std::tuple<int,int> >::iterator it2 = configuration_point_list.begin() ; it2 != configuration_point_list.end(); ++it2){
    	 //length_array[std::distance(configuration_point_list.begin(), it2)] = pow((std::get<0>(*it1) - std::get<0>(*it2)), 2) + pow((std::get<1>(*it1) - std::get<1>(*it2)), 2);
    	//std::cout << "row: " << std::get<0>(*it) << " col: " << std::get<1>(*it) << std::endl;


    	length = sqrt(pow((std::get<0>(*it1) - std::get<0>(*it2)), 2) + pow((std::get<1>(*it1) - std::get<1>(*it2)), 2));
    	//line_segment_check[std::distance(configuration_point_list.begin(), it2)] = check_line_segment(std::get<0>(*it1), std::get<1>(*it1), std::get<0>(*it2), std::get<1>(*it2));

    	 std::cout<< length << ";" ;
    	 point_list.insert(std::pair<int,int>(length, (std::distance(configuration_point_list.begin(), it2))+ 1));
    	 }
     std::cout << std::endl;
     test_ordered_multimap(length_array, point_list, configuration_point_list.size());
     for (int c=0; c < (total_number_of_configurations+2); c++){
    	 if(length_array[c]){ // if node is one of 5 nearest neighbor
    		 if(!check_line_segment(std::get<0>(*it1), std::get<1>(*it1), std::get<0>(configuration_point_list.at(c)), std::get<1>(configuration_point_list.at(c)))){ //if line segment is not free to this close neighbor
    			 length_array[c] = 0;
    		 }
    	 }
    	 std::cout<< length_array[c] << ";" ;
     }
     std::cout << std::endl;
     point_list.clear();
     memcpy(graph[std::distance(configuration_point_list.begin(), it1)], &length_array, sizeof(int) * configuration_point_list.size());
   //std::this_thread::sleep_for (std::chrono::milliseconds(500));
   }
std::cout << "Number of Configuration Points:" << configuration_point_list.size()  << std::endl;

for(int m=0; m<configuration_point_list.size(); m++){
	for(int n=0; n<configuration_point_list.size(); n++){
		std::cout << graph[m][n] << " " ;
	}
	std::cout << std::endl;
}

Shortest_Path(graph, 0, configuration_point_list.size());
int destination = 1;


if((distance[destination] < INT_MAX) && distance[destination] > 0 ){
	std::cout << "Path Found" <<  std::endl;
        path_to_publish.poses.clear();
	printPath(parent, destination);
	std::cout << "Distance is:" << distance[destination] << std::endl;
    if(path_to_publish.poses.size() > 0)
    {
     path_publisher.publish(path_to_publish);
     ROS_INFO("Path is Published with /path topic");
    }
        delete(graph);   
        return true;
}

delete(graph);
return false;
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















