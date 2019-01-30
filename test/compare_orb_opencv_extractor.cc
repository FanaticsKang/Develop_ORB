#include <chrono>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "opencv2/highgui.hpp"
#include "orb_extractor.h"

int main(int argc, char **argv) {
  if (argc < 2) {
    return 0;
  }
  using namespace std::chrono;
  // 初始化
  cv::Mat testMat = cv::imread("/home/kang/Pictures/VIO_PIC/frame0000.jpg",
                               CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat matMask = cv::Mat::ones(testMat.size(), CV_8UC1);
  steady_clock::time_point start, end;
  duration<double> time_span;

  // *******************ORB的特征提取子***************************
  VSLAM::ORBextractor myORB(2000, 1.2f, 8, 20, 7);

  std::vector<cv::KeyPoint> kp;
  cv::Mat desc;

  start = steady_clock::now();
  myORB(testMat, matMask, kp, desc);
  end = steady_clock::now();
  time_span = duration_cast<duration<double>>(end - start);
  std::cout << "ORB EXTRACTOR " << time_span.count() << " seconds."
            << std::endl;

  std::cout << "Finished" << std::endl;
  cv::Mat keypoint_img;
  cv::drawKeypoints(testMat, kp, keypoint_img, cv::Scalar::all(-1), 4);
  // cv::imshow("ORB Drawing", keypoint_img);

  //****************Opencv算法**********************************
  std::vector<cv::KeyPoint> kp2;
  cv::Mat desc2;
  cv::Ptr<cv::Feature2D> detector = cv::ORB::create(
      2000, 1.2, 8, 8, 0, 2, cv::ORB::FAST_SCORE, 8, atoi(argv[1]));

  start = steady_clock::now();
  detector->detectAndCompute(testMat, matMask, kp2, desc2);
  end = steady_clock::now();
  time_span = duration_cast<duration<double>>(end - start);
  std::cout << "OPENCV EXTRACTOR: " << time_span.count() << " seconds."
            << std::endl;

  std::cout << "KeyPoint Size: " << kp2.size() << std::endl;

  cv::Mat keypoint_img2;
  drawKeypoints(testMat, kp2, keypoint_img2, cv::Scalar::all(-1), 4);

  // cv::imshow("Opencv Drawing", keypoint_img2);
  // cv::waitKey(-1);

  // need change the protected to public
  //   myORB.ComputePyramid(testMat);
  //   cv::imshow("view", testMat);
  //   std::cout << "original one" << std::endl;
  //   cv::waitKey(-1);
  //   int i = 0;
  //   for(auto iter = myORB.mvImagePyramid.begin();
  //       iter != myORB.mvImagePyramid.end();
  //       ++iter)
  //   {
  //     cv::imshow("view", *iter);
  //     std::cout << "Pyramid: " << i << std::endl;
  //     cv::waitKey(-1);
  //     ++i;
  //   }
}
