#ifndef VIEW_CLUSTERING_H
#define VIEW_CLUSTERING_H

#include "io_utils.h"

class ViewClustering
{
	const InputDataset& data;
	float x_min, x_max, z_min, z_max;

public:
	std::vector<Cluster> clusters;
	ViewClustering(const InputDataset& input_data) : data(input_data) {};
	void ClusterViews(const int block_size, const int min_points);

private:
	void ComputePointCloudRange();
	void BuildPointCloudGrid(const int block_size, const int num_blocks_x, const int num_blocks_z);
	void FilterPointCloudGrid(const int min_points, const int num_blocks_x, const int num_blocks_z);
	void AssignCamerasToBlock();
};

#endif