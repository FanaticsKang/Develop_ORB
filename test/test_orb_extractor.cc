#include "opencv2/highgui.hpp"
#include "orb_extractor.h"
#include <chrono>
#include <iostream>
#include <opencv2/features2d.hpp>
int main() {
  using namespace std::chrono;
  VSLAM::ORBextractor myORB(2000, 1.2f, 8, 20, 7);
  cv::Mat testMat = cv::imread("/home/kang/Pictures/VIO_PIC/frame0000.jpg",
                               CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat matMask = cv::Mat::ones(testMat.size(), CV_8UC1);

  std::vector<cv::KeyPoint> kp;
  cv::Mat desc;

  myORB(testMat, matMask, kp, desc);
  std::cout << "Size of Key Point: " << kp.size() << std::endl;
  std::cout << "Finished" << std::endl;
  cv::Mat keypoint_img;
  cv::drawKeypoints(testMat, kp, keypoint_img, cv::Scalar::all(-1), 4);
  cv::imshow("ORB Drawing", keypoint_img);
  cv::waitKey(0);
}
