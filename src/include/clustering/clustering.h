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

// 	void ComputeNeighbors(const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0);
// 	bool WriteClustersFiles(const std::string& output_path, const int num_neighbors);
// 	bool WriteColmapFiles(const std::string& output_path);
	
// 	void ComputeNeighborsForCluster(const int i, const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0);
// 	float ComputeViewSelectionScore(const std::vector<Feature>& idx, const int ref_frame, const int ref_sensor, const int src_frame, const int src_sensor, const float sigma_0, const float sigma_1, const float theta_0);
	
// 	bool WriteCamerasFiles(const std::string& path, const int idx);
// 	bool WriteNeighborsFile(const std::string& path, const int idx, const int num_neighbors);
// 	bool WriteImages(const std::string& path, const int idx);

// 	bool WriteColmapCamerasFile(const std::string& path, const int idx);
// 	bool WriteColmapImagesFile(const std::string& path, const int idx);
// 	bool WriteColmapPointsFile(const std::string& path, const int idx);
};

#endif