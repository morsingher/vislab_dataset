#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "data_structures.h"
#include "math_utils.h"

class InputDataset
{
public:

	std::vector<Point> points;
	std::vector<Image> images;
	std::vector<int> filt;

	// Input

	bool LoadPoints(const std::string& filename);
	bool LoadFeatures(const std::string& filename, const int num_cameras);
	bool LoadPoses(const std::string& filename);
	
	// Process

	void FilterPoses(const float min_dist);
	void ComputeDepthRange();
	void BuildFeatureTracks();
};

#endif