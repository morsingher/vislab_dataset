#ifndef CLUSTERING_H
#define CLUSTERING_H

#include "input_dataset.h"

class Clustering
{
	InputDataset& data;
	float x_min, x_max, z_min, z_max;

public:

	std::vector<Cluster> clusters;
	
	Clustering(InputDataset& input_data) : data(input_data) {};
	void ClusterViews(const int block_size, const int min_points, const int min_cameras, const float max_distance);
	void ComputeNeighbors(const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0);
	bool WriteClustersFiles(const std::string& output_path, const int num_neighbors);
	bool WriteColmapFiles(const std::string& output_path);
	void PrintReport();
	
private:

	void ComputePointCloudRange();
	
	void AssignPointsToBlock(const int block_size, const int num_blocks_x);
	void GroupByPoints(const int min_points, const int num_blocks_x);
	void AssignCamerasToBlock(const float max_distance);
	void GroupByCameras(const int min_cameras, const int num_blocks_x);
	
	void ComputeNeighborsForCluster(const int i, const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0);
	float ComputeViewSelectionScore(const std::vector<Feature>& idx, const int ref_frame, const int ref_sensor, const int src_frame, const int src_sensor, const float sigma_0, const float sigma_1, const float theta_0);
	
	bool WriteCamerasFiles(const std::string& path, const int idx);
	bool WriteNeighborsFile(const std::string& path, const int idx, const int num_neighbors);
	bool WriteImages(const std::string& path, const int idx);

	bool WriteColmapCamerasFile(const std::string& path, const int idx);
	bool WriteColmapImagesFile(const std::string& path, const int idx);
	bool WriteColmapPointsFile(const std::string& path, const int idx);
};

#endif