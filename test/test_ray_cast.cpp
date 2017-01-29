/**
 * light_scan_sim test_ray_cast.cpp
 * @brief Test simulating laser rays on an image.
 *
 * @copyright 2017 Joseph Duchesne
 * @author Joseph Duchesne
 */

#include "light_scan_sim/ray_cast.h"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// The fixture for testing class RayCast
class RayCastTest : public ::testing::Test {};

TEST_F(RayCastTest, TestTest) {
  RayCast rc;
  cv::Point start, end, hit;

  // Set up the raycast with a very simple map
  cv::Mat mat = cv::Mat::zeros(20, 20, CV_8UC1); 
  cv::rectangle( mat, cv::Point( 10, 0 ), cv::Point( 20, 20), 255, CV_FILLED);
  rc.SetMap(mat, 1.0);

  // Test tracing from empty space into wall
  start = cv::Point(5,5);
  end = cv::Point(15,5);
  EXPECT_TRUE(rc.Trace(start, end, hit));
  EXPECT_NEAR(hit.x, 10, 1e-5);
  EXPECT_NEAR(hit.y, 5, 1e-5);
  
  // Test tracing out of map into empty space
  start = cv::Point(5,5);
  end = cv::Point(-5,5);
  EXPECT_FALSE(rc.Trace(start, end, hit));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
