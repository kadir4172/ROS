#include <gtest/gtest.h>
#include <climits>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//#include "og_help/dtf.h"


TEST(ImageSize,ShouldPass){

    //! I would STRONGLY suggest to write two functions
    //! image2global and global2image and Unit test them
    //! The functions would allow to go from image space
    //! To Global coordinate system and vice versa

    //! The below code does not do this, it merley sets up the framework
    //! For testing and allows accessing an OPENCV image

    double mapSize=20.0;
    double resolution=0.1;

    int pixels = (int) mapSize / resolution;

    // Create an OgMap which is a grayscale (mono) image of size pixels
    cv::Mat image = cv::Mat::zeros(pixels,pixels, CV_8UC1);

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
