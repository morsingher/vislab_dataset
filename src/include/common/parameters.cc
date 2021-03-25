#include "parameters.h"

bool Parameters::Load(const char* filename)
{
	FILE* fp = fopen(filename, "r");
	if (fp == nullptr)
	{
		std::cout << "Couldn't open the configuration file." << std::endl;
		return false;
	}

 	char read_buffer[65536];
	rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer)); 
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	// File paths

	project_path = d["project_path"].GetString();
	input_folder = project_path + d["input_folder"].GetString();
	images_folder = d["images_folder"].GetString();
	cameras_folder = input_folder + "cameras/";
	output_folder = project_path + d["output_folder"].GetString();
	poses_file = input_folder + d["poses_file"].GetString();
	points_file = input_folder + d["points_file"].GetString();
	features_file = input_folder + d["features_file"].GetString();

	// Input processing

	num_cameras = d["num_cameras"].GetInt();
	min_dist = d["min_dist"].GetDouble();
	min_depth = d["min_depth"].GetDouble();
	max_depth = d["max_depth"].GetDouble();

	// Clustering 

	block_size = d["block_size"].GetInt();
	min_points = d["min_points"].GetInt();
	min_cameras = d["min_cameras"].GetInt();
	max_distance = d["max_distance"].GetDouble();

	// // Neighbors

	// num_neighbors = d["num_neighbors"].GetInt();
	// theta_0 = static_cast<float>(d["theta_0"].GetDouble());
	// sigma_0 = static_cast<float>(d["sigma_0"].GetDouble());
	// sigma_1 = static_cast<float>(d["sigma_1"].GetDouble());

	return true;
}