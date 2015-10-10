#ifndef VIS_IMAGE_PYRAMID_H_
#define VIS_IMAGE_PYRAMID_H_

class ImagePyramid {
 public:
  ImagePyramid();
  explicit ImagePyramid(const cv::Mat& img) { Init(img); }

 private:
  void Init(const cv::Mat& img);

  std::vector<cv::Mat> pyr_;
};

#endif  // VIS_IMAGE_PYRAMID_H_
