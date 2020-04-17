//
// Created by dji on 20-4-16.
//

#ifndef PROJECT_CAMERA_MODEL_UTILS_H
#define PROJECT_CAMERA_MODEL_UTILS_H

#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>

class CameraModelUtils {
  public:
  CameraModelUtils(){};
  void InitWideAngleCameraMap(cv::Size image_size, const cv::Mat& K, const cv::Mat& D, const cv::Mat K_rect, const cv::Mat& rot, cv::Mat* map1, cv::Mat* map2);

 private:
  std::string cam_model;

};

#endif //PROJECT_CAMERA_MODEL_UTILS_H
