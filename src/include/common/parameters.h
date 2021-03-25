#ifndef MVC_PARAMETERS_H
#define MVC_PARAMETERS_H

#include "common.h"

class Parameters
{
public:

	// File paths

	std::string project_path;
	std::string input_folder;
	std::string images_folder;
	std::string cameras_folder;
	std::string output_folder;
	std::string poses_file;
	std::string points_file;
	std::string features_file;

	int num_cameras;

	// Keyframe selection

	double min_dist;

	// // Clustering

	// int block_size;
	// int min_points;
	// int min_cameras;
	// float max_distance;

	// // Neighbors

	// int num_neighbors;
	// float theta_0;
	// float sigma_0;
	// float sigma_1;

	bool Load(const char* filename);
};

#endif