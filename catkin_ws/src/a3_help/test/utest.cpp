#include <gtest/gtest.h>
#include <climits>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>


cv::Mat image;
geometry_msgs::Pose pose;
double mapSize=30.0;
double resolution_=0.1;

void global2image(double global_x, double global_y, int* local_x, int* local_y){
//! This utility function will be used to convert global position data to local OGMAP pixel values	
	
  //calculate local_x and local_y with respect to robot's global position since OGMAP is centered with robot itself
  *local_y = (int)((double)image.rows/2 + (double)(pose.position.y - global_y) / resolution_);
  *local_x = (int)((double)image.cols/2 - (double)(pose.position.x - global_x) / resolution_);
}

void image2global(double* global_x, double* global_y, int local_x, int local_y){
//! This utility function will be used to convert local position data to global map coordinates frame
	
  //calculate global_x and global_y with respect to robot's global position since OGMAP is centered with robot itself
  *global_y = ( (image.rows/2) - (double)local_y +  ((double)(pose.position.y) / resolution_)) * resolution_;
  *global_x = (-(image.cols/2) + (double)local_x +  ((double)(pose.position.x) / resolution_)) * resolution_;
}

TEST(ImageSize,ShouldPass){

    //! This unit test routine randomly selects 5000000 global robot positions and convert them local pixel coordinates vica-versa

    //Variables to store converted values
	int local_x;
    int local_y;
    double global_x;
    double global_y;
    
    //Pixel count of actual map
    int pixels = (int) mapSize / resolution_;

    //Create an OgMap which is a grayscale (mono) image of size pixels
    image = cv::Mat::zeros(pixels,pixels, CV_8UC1);

    //Initialize random seed
    srand (time(NULL));

    for(int i = 0 ; i < 5000000 ; i++){
      //Assign robot global position position randomly
      pose.position.x = rand() % (int)mapSize; 
      pose.position.y = rand() % (int)mapSize;
      
      //Convert this position to local pixel map, and check whether if its on the center of map
      global2image((double)pose.position.x, (double)pose.position.y, &local_x, &local_y);
      
      //Since OGMAP center coincides with robot position, its expected to calculate center point of OGMAP
      ASSERT_EQ(local_x, image.cols/2);
      ASSERT_EQ(local_y, image.rows/2);
      
      //Convert this center position to the global location of robot
      image2global(&global_x, &global_y, local_x, local_y);
      
      //Its expected to calculate randomly selected global position of robot 
      ASSERT_EQ(global_x, pose.position.x);
      ASSERT_EQ(global_y, pose.position.y);
    }
    
    // Let's check map size compared to allocation, just in case
    ASSERT_EQ(pixels, image.rows);
    ASSERT_EQ(pixels, image.cols);

    // Let's check the map is allocated all zeros
    ASSERT_EQ(0,image.at<uchar>(0,0));

    // Draw a link from top left to botom right corner
    cv::line(image,cv::Point(0,0),cv::Point(pixels,pixels),cv::Scalar(255,255,255),1);

    // Let's check the centre is now white (255)
    ASSERT_EQ(255,image.at<uchar>(pixels/2,pixels/2));

}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
