#ifndef VIS_KEYFRAME_H_
#define VIS_KEYFRAME_H_

#include <Eigen/Dense>
#include <vector>

#include "opencv2/core/core.hpp"
#include "vis/image_pyramid.h"

class Frame {
 public:
  void Init(const ImagePyramid* pyr);

  // Align existing keypoints to a new image using Lucas-Kanade
  void AlignKeypoints(const ImagePyramid& pyramid, std::vector<Vector2f> *out);

 private:
  // image pyramid
  const ImagePyramid *pyr_;
  // each pyramid level has a separate set of keypoints
  std::vector<std::vector<Vector2f> > keypoints_;
};

#endif  // VIS_KEYFRAME_H_
