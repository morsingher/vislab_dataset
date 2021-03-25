#ifndef MVC_INPUT_DATASET_H
#define MVC_INPUT_DATASET_H

#include "parameters.h"

class InputDataset
{
public:

	InputDataset(const Parameters& params);

	bool Load();
	void FilterPoses(const double min_dist);

	std::vector<Point> points;
	std::vector<Frame> images;
	std::vector<int> idx_filt;

	int num_frames, num_cameras, num_points;

private:

	// Input

	bool LoadPoints();
	bool LoadFeatures();
	bool LoadPoses();
	bool ReadCalibrationFile(const int cam_id);
	void SetFilenames();

	// Process

	// void ComputeDepthRange();
	// void BuildFeatureTracks();

	std::string cameras_folder;
	std::string images_folder;
	std::string poses_file;
	std::string points_file;
	std::string features_file;

	const std::map<int, std::string> cam_dictionary = {
		{0, "FC"},
		{1, "BC"},
		{2, "FL"},
		{3, "FR"},
		{4, "BL"},
		{5, "BR"},
	};
};

#endif