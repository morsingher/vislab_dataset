#include "input_dataset.h"
#include "view_clustering.h"
#include "parameters.h"

int main(int argc, char** argv) 
{
	std::cout << std::endl;

	// Check if the files has been provided

	if (argc != 2)
	{
		std::cout << "Usage " << argv[0] << " PATH_TO_CONFIG_FILE" << std::endl;
		return EXIT_FAILURE;
	}

	// Load parameters

	std::cout << "Loading parameters..." << std::endl;
	Parameters params;
	if (!params.Load(argv[1]))
	{
		std::cout << "Failed to load parameters" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Done!" << std::endl << std::endl;;

	// Load input dataset

	std::cout << "Loading input Ambarella dataset..." << std::endl;

	InputDataset dataset;
	dataset.num_cameras = params.num_cameras;

	// state.bap file (points)

	if(!dataset.LoadPoints(params.points_file))
	{
		std::cout << "Failed to load points" << std::endl;
		return EXIT_FAILURE;
	}

	// state.bin file (features)

	if (!dataset.LoadFeatures(params.features_file))
	{
		std::cout << "Failed to load features" << std::endl;
		return EXIT_FAILURE;
	}

	// outputPose_correct.txt file (poses)

	if (!dataset.LoadPoses(params.poses_file))
	{
		std::cout << "Failed to load poses" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "Done!" << std::endl << std::endl;

	// Filter out redundant poses

	std::cout << "Filtering poses by selecting keyframes..." << std::endl;
	dataset.FilterPoses(params.min_difference);
	std::cout << "Done! Selected " << dataset.filt.size() << " keyframes" << std::endl << std::endl;

	// Compute depth range

	std::cout << "Computing depth range for selected keyframes..." << std::endl;
	dataset.ComputeDepthRange();
	std::cout << "Done!" << std::endl << std::endl;

	// Assign images to each point and remove useless points

	std::cout << "Assigning keyframes to each visible point..." << std::endl;
	dataset.BuildFeatureTracks();
	std::cout << "Done! There are " << dataset.points.size() << " visible points" << std::endl << std::endl;

	// Cluster points and cameras

	std::cout << "Clustering points and cameras..." << std::endl;
	ViewClustering view_clustering(dataset);
	view_clustering.ClusterViews(params.block_size, params.min_points, params.min_cameras, params.max_distance);
	std::cout << "Done! Built " << view_clustering.clusters.size() << " clusters" << std::endl << std::endl;

	// Compute neighbors

	std::cout << "Computing neighbors for each cluster..." << std::endl;
	view_clustering.ComputeNeighbors(params.num_neighbors, params.sigma_0, params.sigma_1, params.theta_0);
	std::cout << "Done!" << std::endl << std::endl;

	// Write files in COLMAP format

	std::cout << "Saving results in COLMAP format..." << std::endl;
	if (!view_clustering.WriteColmapFiles(params.output_folder))
	{
		std::cout << "Failed to save results in COLMAP format" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Done!" << std::endl << std::endl;

	// Write files in standard format

	std::cout << "Saving results in standard format..." << std::endl;
	if (!view_clustering.WriteClustersFiles(params.output_folder, params.num_neighbors))
	{
		std::cout << "Failed to save results in standard format" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Done!" << std::endl << std::endl;

	return EXIT_SUCCESS;
}