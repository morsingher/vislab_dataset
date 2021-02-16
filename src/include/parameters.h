#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "data_structures.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"

class Parameters
{
public:

	// File paths

	std::string project_path;
	std::string input_folder;
	std::string output_folder;
	std::string poses_file;
	std::string points_file;
	std::string features_file;

	// Keyframe selection

	float min_difference;

	// Clustering

	int block_size;
	int min_points;
	int min_cameras;
	float max_distance;

	// Neighbors

	int num_neighbors;
	float theta_0;
	float sigma_0;
	float sigma_1;

	bool Load(const char* params_file);
};

#endif