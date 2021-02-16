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

	auto begin = std::chrono::steady_clock::now();

	Parameters params;
	if (!params.Load(argv[1]))
	{
		std::cout << "Failed to load parameters" << std::endl;
		return EXIT_FAILURE;
	}

	InputDataset dataset;

	// state.bap file

	if(!dataset.LoadPoints(params.points_file))
	{
		std::cout << "Failed to load points" << std::endl;
		return EXIT_FAILURE;
	}

	// state.bin file

	if (!dataset.LoadFeatures(params.features_file))
	{
		std::cout << "Failed to load features" << std::endl;
		return EXIT_FAILURE;
	}

	// outputPose_correct.txt file

	if (!dataset.LoadPoses(params.poses_file))
	{
		std::cout << "Failed to load poses" << std::endl;
		return EXIT_FAILURE;
	}

	auto end = std::chrono::steady_clock::now();
	auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
	std::cout << "Loaded parameters and dataset in " << elapsed_time << " s." << std::endl;

	// Filter out redundant poses

	begin = std::chrono::steady_clock::now();

	dataset.FilterPoses(params.min_difference);

	std::cout << std::endl;
	end = std::chrono::steady_clock::now();
	elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
	std::cout << "Filtered poses in " << elapsed_time << " s." << std::endl;

	// Assign images to each point and remove useless points

	begin = std::chrono::steady_clock::now();

	dataset.BuildFeatureTracks();
	const auto lambda_size = [&](const Point& p) { return p.frame_idx.size() == 0; };
	dataset.points.erase(std::remove_if(dataset.points.begin(), dataset.points.end(), lambda_size), dataset.points.end());

	std::cout << std::endl;
	end = std::chrono::steady_clock::now();
	elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
	std::cout << "Built feature tracks in " << elapsed_time << " s." << std::endl;

	// Cluster points and cameras

	begin = std::chrono::steady_clock::now();
	std::cout << std::endl;

	ViewClustering view_clustering(dataset);
	view_clustering.ClusterViews(params.block_size, params.min_points, params.min_cameras, params.max_distance);

	std::cout << std::endl;
	end = std::chrono::steady_clock::now();
	elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
	std::cout << "Clustered cameras and points in " << elapsed_time << " s." << std::endl;

	// Compute depth range

	begin = std::chrono::steady_clock::now();

	dataset.ComputeDepthRange();

	std::cout << std::endl;
	end = std::chrono::steady_clock::now();
	elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
	std::cout << "Computed depth range in " << elapsed_time << " s." << std::endl;

	// Compute neighbors

	begin = std::chrono::steady_clock::now();
	std::cout << std::endl;

	view_clustering.ComputeNeighbors(params.num_neighbors, params.theta_0, params.sigma_0, params.sigma_1);

	std::cout << std::endl;
	end = std::chrono::steady_clock::now();
	elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count() / 1e9;
	std::cout << "Computed neighbors in " << elapsed_time << " s." << std::endl;

	// Write cameras to file as required by PatchMatchNet

	// std::string path("/home/marco/vislab_dataset/results/cams_1/");
	// if (!dataset.WriteCameraFiles( path))
	// {
	// 	std::cout << "Failed to write cameras" << std::endl;
	// 	return EXIT_FAILURE;
	// }

	// Write view selection results

	// path = std::string("/home/marco/vislab_dataset/results/");
	// if (!dataset.WriteNeighborsFile( path))
	// {
	// 	std::cout << "Failed to write neighbors" << std::endl;
	// 	return EXIT_FAILURE;
	// }

	std::cout << std::endl;

	return EXIT_SUCCESS;
}