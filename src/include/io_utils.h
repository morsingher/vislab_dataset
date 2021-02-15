#ifndef IO_UTILS_H
#define IO_UTILS_H

#include "data_structures.h"
#include "math_utils.h"

class InputDataset
{
public:

	std::vector<Point> points;
	std::vector<Frame> images;

	// Input

	bool LoadPoints(const std::string& filename);
	bool LoadFeatures(const std::string& filename);
	bool LoadPoses(const std::string& filename);
	
	// Process

	std::vector<int> FilterPoses(const float min_dist);
	void BuildFeatureTracks(const std::vector<int>& filt);

	void ComputeDepthRange(const std::vector<int>& filt);
	void ComputeNeighbors(const std::vector<int>& filt, const float sigma_0, const float sigma_1, const float theta_0);
	
	// Output

	bool WriteCameraFiles(const std::vector<int>& filt, const std::string& path);
	bool WriteNeighborsFile(const std::vector<int>& filt, const std::string& path);

private:
	float ComputeViewSelectionScore(const std::vector<int>& idx, 
									const int ref, 
									const int src,
									const float sigma_0, 
									const float sigma_1, 
									const float theta_0);	
};

#endif