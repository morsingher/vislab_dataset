#ifndef MVC_COMMON_H
#define MVC_COMMON_H

// C++ headers

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <chrono>
#include <unordered_map>
#include <set>
#include <cstdlib>
#include <thread>

// OpenCV header

#include <opencv2/opencv.hpp>

// Linux headers (for mkdir)

#include <sys/stat.h>
#include <sys/types.h>

// RapidJson headers

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"

struct Point
{
	double x, y, z;
	std::vector<std::pair<int, int>> image_idx; // (UUID, feature index)
	// int r, g, b; // TODO
	// float error; // TODO
	Point() : x(0.0), y(0.0), z(0.0) {};
	Point(const double _x, const double _y, const double _z) : x(_x), y(_y), z(_z) {};
};

struct Feature
{
	uint32_t point_idx;
	cv::Point2f left, right;
	// bool operator<(const Feature& f)
	// {
	// 	return point_idx < f.point_idx;
	// }
};

// struct Neighbor
// {
// 	int uuid;
// 	float score;
// 	Neighbor() : uuid(-1), score(0.0f) {}; 
// 	Neighbor(const int i, const float s) : uuid(i), score(s) {};
// };

struct Image
{
	std::string filename;
	int width, height;
	cv::Mat_<float> K, R, t;
	std::vector<Feature> features;
	float min_depth = 0.0f, max_depth = 1.0f;
	// std::vector<Neighbor> neighbors;
};

typedef std::vector<Image> Frame;

struct Cluster
{
	std::vector<int> point_idx;
	// std::set<int> camera_idx; // UUID
	// std::unordered_map<int, std::vector<Neighbor>> neighbors;
};

#endif