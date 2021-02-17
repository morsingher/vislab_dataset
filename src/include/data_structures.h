#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <chrono>
#include <unordered_map>
#include <cstdlib>
#include <thread>
#include <opencv2/opencv.hpp>

#define WITHOUT_NUMPY
#include "matplotlibcpp.h"

struct Point
{
	double x, y, z;
	std::vector<int> frame_idx;
	Point() : x(0.0f), y(0.0f), z(0.0f) {};
	Point(const float x_coord, const float y_coord, const float z_coord) : x(x_coord), y(y_coord), z(z_coord) {};
};

struct Feature
{
	uint32_t point_idx;
	cv::Point2f left, right;
	bool operator<(const Feature& f)
	{
		return point_idx < f.point_idx;
	}
};

struct Neighbor
{
	int idx;
	float score;
	Neighbor() : idx(-1), score(0.0f) {}; 
	Neighbor(const int i, const float s) : idx(i), score(s) {};
};

struct Frame
{
	std::string filename;
	float baseline;
	int width, height;
	cv::Mat_<float> K, R, t;
	std::vector<Feature> features;
	float min_depth, max_depth;
	std::vector<Neighbor> neighbors;
};

struct Cluster
{
	std::vector<int> point_idx;
	std::vector<int> camera_idx;
	std::unordered_map<int, std::vector<Neighbor>> neighbors;
};

#endif