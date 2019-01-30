/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University
 * of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ORB_EXTRACTOR_H
#define ORB_EXTRACTOR_H

#include <opencv/cv.h>
#include <list>
#include <vector>

namespace VSLAM {

/**
 * @brief 提取出来的特征点, 指明了某一个区域内所有的特征点
 * UL, UR, BL, BR分别表示UpLeft, UpRight, BottomLeft, BottomRight
 */
class ExtractorNode {
 public:
  ExtractorNode() : bNoMore(false) {}

  void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3,
                  ExtractorNode &n4);
  //这个区域所有的特征点
  std::vector<cv::KeyPoint> vKeys;
  cv::Point2i UL, UR, BL, BR;
  std::list<ExtractorNode>::iterator lit;
  //对于每个 node 而言，若其只有一个特征点，bNoMore 为
  // true，表明其不用再继续划分。
  bool bNoMore;
};

class ORBextractor {
 public:
  enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

  /**
   * @brief 构造函数
   *
   * @param nfeatures 最大特征点个数;
   * @param scaleFactor 金字塔比例因子, opencv中默认是1.2f;
   * @param nlevels 层数, opencv和orbslam均使用8;
   * @param iniThFAST 角点阈值. orbslam使用20;
   * @param minThFAST 未检测到角点后降低的阈值, orbslam使用7;
   */
  ORBextractor(int nfeatures, float scaleFactor, int nlevels, int iniThFAST,
               int minThFAST);

  ~ORBextractor() {}

  // Compute the ORB features and descriptors on an image.
  // ORB are dispersed on the image using an octree.
  // Mask is ignored in the current implementation.
  void operator()(cv::InputArray image, cv::InputArray mask,
                  std::vector<cv::KeyPoint> &keypoints,
                  cv::OutputArray descriptors);

  int inline GetLevels() { return nlevels; }

  float inline GetScaleFactor() { return scaleFactor; }

  std::vector<float> inline GetScaleFactors() { return mvScaleFactor; }

  std::vector<float> inline GetInverseScaleFactors() {
    return mvInvScaleFactor;
  }

  std::vector<float> inline GetScaleSigmaSquares() { return mvLevelSigma2; }

  std::vector<float> inline GetInverseScaleSigmaSquares() {
    return mvInvLevelSigma2;
  }

  /**
   * @brief 图像金字塔的数量在初始化的时候设置完成
   *
   */
  std::vector<cv::Mat> mvImagePyramid;

 protected:
  // public:
  /**
   * @brief 计算图像金子塔, 输出为mvImagePyramid.
   *
   * @param image 输入图像
   */
  void ComputePyramid(cv::Mat image);
  /**
   * @brief 将图像分成若干格, 每一格的提取特征点
   *
   * @param allKeypoints 每一层, 每一格的特征点
   */
  void ComputeKeyPointsOctTree(
      std::vector<std::vector<cv::KeyPoint>> &allKeypoints);
  /**
   * @brief ...
   *
   * @param vToDistributeKeys 特征点的集合
   * @param minX, maxX, minY, maxY 指明了图像的边界.
   * @param nFeatures 特征点的个数, 每一层的特征点个数不同.(构造函数中创建)
   * @param level 指明这一层
   * @return std::vector< cv::KeyPoint >
   */
  std::vector<cv::KeyPoint> DistributeOctTree(
      const std::vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
      const int &maxX, const int &minY, const int &maxY, const int &nFeatures,
      const int &level);
  //开始用一大堆数字进行初始化
  std::vector<cv::Point> pattern;

  int nfeatures;
  double scaleFactor;
  int nlevels;
  int iniThFAST;
  int minThFAST;

  //每一层的特征点个数, 由构造函数第二部分完成.
  std::vector<int> mnFeaturesPerLevel;

  // 跟特征点的描述子有关.
  std::vector<int> umax;

  // 以下参数和金字塔尺度有关, 大小由nlevel决定.
  // 在构造函数中初始化
  // mvScaleFactor[0]     [1]             [2]
  //      1.0          scaleFactor   scaleFactor^2
  //
  // mvLevelSigma2[i] = mvScaleFactor^2;
  std::vector<float> mvScaleFactor;
  std::vector<float> mvInvScaleFactor;  // = 1/mvScaleFactor;
  std::vector<float> mvLevelSigma2;
  std::vector<float> mvInvLevelSigma2;  // = 1/mvLevelSigma2;
};

}  // namespace VSLAM

#endif  // ORB_EXTRACTOR_H
