#include <chrono>
#include <iostream>
#include <opencv2/features2d.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "orb_extractor.h"
int main() {
  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;  // = 1/mvScaleFactor;
  std::vector<cv::Mat> mvImagePyramid;

  float scaleFactor = 1.2f;  //金字塔每层
  int nlevels = 8;           //金字塔总层数
  // ORB提取子
  mvScaleFactor.resize(nlevels);
  mvInvScaleFactor.resize(nlevels);
  mvScaleFactor[0] = 1.0f;
  mvInvScaleFactor[0] = 1.0f;
  for (int i = 1; i < nlevels; i++) {
    mvScaleFactor[i] = mvScaleFactor[i - 1] * scaleFactor;
    mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
  }
  for (int i = 0; i < nlevels; i++) {
    std::cout << "mvScaleFactor[" << i << "]: " << mvScaleFactor[i];
    std::cout << "; mvInvScaleFactor[" << i << "]: " << mvInvScaleFactor[i];
    std::cout << std::endl;
  }
  cv::Mat image = cv::imread("/home/kang/Pictures/VIO_PIC/frame0000.jpg",
                             CV_LOAD_IMAGE_GRAYSCALE);
  mvImagePyramid.resize(nlevels);
  for (int level = 0; level < nlevels; ++level) {
    float scale = mvInvScaleFactor[level];  //获得因子
    cv::Size sz(cvRound((float)image.cols * scale),
                cvRound((float)image.rows * scale));
    if (level != 0) {
      cv::resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0,
                 cv::INTER_LINEAR);
    } else {
      cv::resize(image, mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
    }
    cv::imshow("Pyramid: " + std::to_string(level), mvImagePyramid[level]);
    cv::waitKey(500);
  }
  cv::waitKey(0);
}