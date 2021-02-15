#include "io_utils.h"
#include "view_clustering.h"

namespace plt = matplotlibcpp;

int main(int argc, char** argv) 
{
	std::cout << std::endl;

	// Check if the files has been provided

	if (argc != 4)
	{
		std::cout << "Usage " << argv[0] << " PATH_TO_POINTS_FILE PATH_TO_FEATURES_FILE PATH_TO_POSES_FILE" << std::endl;
		return EXIT_FAILURE;
	}

	InputDataset dataset;

	// state.bap file

	const std::string points_file(argv[1]);

	if(!dataset.LoadPoints(points_file))
	{
		std::cout << "Failed to load points" << std::endl;
		return EXIT_FAILURE;
	}

	// state.bin file

	const std::string features_file(argv[2]);

	if (!dataset.LoadFeatures(features_file))
	{
		std::cout << "Failed to load features" << std::endl;
		return EXIT_FAILURE;
	}

	// outputPose_correct.txt file

	const std::string poses_file(argv[3]);

	if (!dataset.LoadPoses(poses_file))
	{
		std::cout << "Failed to load poses" << std::endl;
		return EXIT_FAILURE;
	}

	// Filter out redundant poses

	std::vector<int> idx_filtered = dataset.FilterPoses(1.0f);
	dataset.BuildFeatureTracks(idx_filtered);

	const auto lambda_size = [&](const Point& p) { return p.frame_idx.size() == 0; };
	dataset.points.erase(std::remove_if(dataset.points.begin(), dataset.points.end(), lambda_size), dataset.points.end());
	std::cout << "Points seen by filtered frame and central camera: " << dataset.points.size() << std::endl;

	std::cout << std::endl;

	ViewClustering view_clustering(dataset);
	view_clustering.ClusterViews(20, 100);

	// Compute depth range

	// dataset.ComputeDepthRange(idx_filtered);

	// Compute neighbors

	// dataset.ComputeNeighbors(idx_filtered, 1.0, 10.0, 5.0);

	// Write cameras to file as required by PatchMatchNet

	// std::string path("/home/marco/vislab_dataset/results/cams_1/");
	// if (!dataset.WriteCameraFiles(idx_filtered, path))
	// {
	// 	std::cout << "Failed to write cameras" << std::endl;
	// 	return EXIT_FAILURE;
	// }

	// Write view selection results

	// path = std::string("/home/marco/vislab_dataset/results/");
	// if (!dataset.WriteNeighborsFile(idx_filtered, path))
	// {
	// 	std::cout << "Failed to write neighbors" << std::endl;
	// 	return EXIT_FAILURE;
	// }

	std::cout << "Exit" << std::endl;

	return EXIT_SUCCESS;
}