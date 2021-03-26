#ifndef MVC_CLUSTERING_H
#define MVC_CLUSTERING_H

#include "input_dataset.h"

class Clustering
{
public:
	Clustering(const Parameters& params, InputDataset& data);
	void ComputeClusters();

	std::vector<Cluster> clusters;

private:

	const Parameters& params;
	InputDataset& data;
	double x_min, x_max, z_min, z_max;
	int num_blocks_x, num_blocks_z;

	void ComputePointCloudRange();
	void AssignPointsToBlock();
	void GroupByPoints();

	void AssignCamerasToBlock();
	void GroupByCameras();

	void PrintReport();
	int CountNonEmptyClusters();

// 	void ComputeNeighbors();
// 	void ComputeNeighborsForCluster(const int i);
// 	float ComputeViewSelectionScore(const std::vector<Feature>& idx, const int ref_frame, const int ref_sensor, const int src_frame, const int src_sensor);
};

#endif