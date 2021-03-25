#include "clustering.h"

int main(int argc, char** argv) 
{
	std::cout << std::endl;

	// Check if the configuration file has been provided

	if (argc != 2)
	{
		std::cout << "Usage " << argv[0] << " PATH_TO_CONFIG_FILE" << std::endl;
		return EXIT_FAILURE;
	}

	// Load parameters

	std::cout << "Loading parameters from: " << std::string(argv[1]) << std::endl;
	Parameters params;
	if (!params.Load(argv[1]))
	{
		std::cout << "Failed to load parameters!" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Done!" << std::endl << std::endl;

	// Load input dataset

	std::cout << "Loading input dataset..." << std::endl;
	InputDataset dataset(params);
	if (!dataset.Load())
	{
		std::cout << "Failed to load input data!" << std::endl;
		return EXIT_FAILURE;
	}
	std::cout << "Done!" << std::endl << std::endl;

	// Multi-view clustering

	std::cout << "Starting multi-view clustering algorithm..." << std::endl;
	Clustering mvc(params, dataset);
	mvc.ComputeClusters();
	std::cout << "Done!" << std::endl << std::endl;

	// Write files

	return EXIT_SUCCESS;
}