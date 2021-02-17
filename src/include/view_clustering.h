#ifndef VIEW_CLUSTERING_H
#define VIEW_CLUSTERING_H

#include "input_dataset.h"

class ViewClustering
{
	const InputDataset& data;
	float x_min, x_max, z_min, z_max;

public:

	std::vector<Cluster> clusters;
	
	ViewClustering(const InputDataset& input_data) : data(input_data) {};

	void ClusterViews(const int block_size, const int min_points, const int min_cameras, const float max_distance);
	
	void ComputeNeighbors(const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0);
	
	bool WriteClustersFiles(const std::string& output_path);

	void PlotClusters();
	
private:

	void ComputePointCloudRange();
	
	void AssignPointsToBlock(const int block_size, const int num_blocks_x);
	void GroupByPoints(const int min_points, const int num_blocks_x, const int num_blocks_z);
	void AssignCamerasToBlock(const float max_distance);
	void GroupByCameras(const int min_cameras, const int num_blocks_x, const int num_blocks_z);
	
	void ComputeNeighborsForCluster(const int i, const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0);
	float ComputeViewSelectionScore(const std::vector<int>& idx, const int ref, const int src, const float sigma_0, const float sigma_1, const float theta_0);
	
	bool WriteCamerasFiles(const std::string& path, const int idx);
	bool WriteNeighborsFile(const std::string& path, const int idx);
};

#endif