#ifndef MVC_MATH_UTILS_H
#define MVC_MATH_UTILS_H

#include "common.h"

float ComputePoseDistance(const cv::Mat_<float>& t1, const cv::Mat_<float>& t2);
Point TransformPointFromWorldToCam(const cv::Mat_<float>& R, const cv::Mat_<float>& t, const Point& p);

// cv::Point2f ObservePoint(const Point& p, const cv::Mat_<float>& K);

// Point ProjectPointAtDepth(const cv::Point2f& pixel, const cv::Mat_<float>& K, const float depth);
// Point TransformPointFromCamToWorld(const cv::Mat_<float>& R, const cv::Mat_<float>& t, const Point& p);
// float ComputeTriangulationAngle(const Point& p, const cv::Mat_<float>& t_ref, const cv::Mat_<float>& t_src);
// Point TransformPointFromFLUToRDF(const Point& p);

// typedef std::array<float, 4> Quaternion;
// Quaternion QuaternionFromRotationMatrix(const cv::Mat_<float>& R);

#endif