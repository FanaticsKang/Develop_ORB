#include <chrono>
#include <iostream>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "orb_extractor.h"
int main() {
  cv::Mat srcMat = cv::imread("/home/kang/Pictures/VIO_PIC/frame0000.jpg",
                              CV_LOAD_IMAGE_GRAYSCALE);
  cv::Mat dstMat;
  cv::GaussianBlur(srcMat, dstMat, cv::Size(7, 7), 2, 2,
                   cv::BORDER_REFLECT_101);

  cv::imshow("Source", srcMat);
  cv::imshow("Target", dstMat);
  cvWaitKey(0);
}