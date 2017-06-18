#include <gtest/gtest.h>
#include <climits>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
//#include "og_help/dtf.h"

cv::Mat image;
geometry_msgs::Pose pose;
double mapSize=30.0;
double resolution_=0.1;

void global2image(double global_x, double global_y, int* local_x, int* local_y){
		*local_y = (int)((double)image.rows/2 + (double)(pose.position.y - global_y) / resolution_);
		*local_x = (int)((double)image.cols/2 - (double)(pose.position.x - global_x) / resolution_);
}
void image2global(double* global_x, double* global_y, int local_x, int local_y){
	*global_y = ( (image.rows/2) - (double)local_y +  ((double)(pose.position.y) / resolution_)) * resolution_;
		*global_x = (-(image.cols/2) + (double)local_x +  ((double)(pose.position.x) / resolution_)) * resolution_;

}
TEST(ImageSize,ShouldPass){

    //! I would STRONGLY suggest to write two functions
    //! image2global and global2image and Unit test them
    //! The functions would allow to go from image space
    //! To Global coordinate system and vice versa

    //! The below code does not do this, it merley sets up the framework
    //! For testing and allows accessing an OPENCV image

    int local_x;
    int local_y;
    double global_x;
    double global_y;
    int pixels = (int) mapSize / resolution_;

   // Create an OgMap which is a grayscale (mono) image of size pixels
    image = cv::Mat::zeros(pixels,pixels, CV_8UC1);

    /* initialize random seed: */
  srand (time(NULL));
for(int i = 0 ; i < 5000000 ; i++){
    pose.position.x = rand() % (int)mapSize; 
    pose.position.y = rand() % (int)mapSize;
    global2image((double)pose.position.x, (double)pose.position.y, &local_x, &local_y);
    ASSERT_EQ(local_x, image.cols/2);
    ASSERT_EQ(local_y, image.rows/2);
    image2global(&global_x, &global_y, local_x, local_y);
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
