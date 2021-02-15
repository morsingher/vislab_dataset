#include "view_clustering.h"

void ViewClustering::ComputePointCloudRange()
{
	x_min = std::numeric_limits<float>::max();
	x_max = -x_min;

	z_min = std::numeric_limits<float>::max();
	z_max = -z_min;

	for (const auto& p : data.points)
	{
		const float x = p.x;
		const float y = p.y;
		const float z = p.z;

		if (x < x_min)
			x_min = x;
		if (x > x_max)
			x_max = x;

		if (z < z_min)
			z_min = z;
		if (z > z_max)
			z_max = z;
	}

	std::cout << "Point cloud range on X: (" << x_min << ", " << x_max << ")" << std::endl;
	std::cout << "Point cloud range on Z: (" << z_min << ", " << z_max << ")" << std::endl;
}

void ViewClustering::ClusterViews(const int block_size, const int min_points)
{
	ComputePointCloudRange();

	const int num_blocks_x = std::ceil(std::abs(x_max - x_min) / block_size);
	const int num_blocks_z = std::ceil(std::abs(z_max - z_min) / block_size);
	clusters.resize(num_blocks_x * num_blocks_z);

	std::cout << "Building a grid with size " << num_blocks_x << " x " << num_blocks_z << std::endl;

	BuildPointCloudGrid(block_size, num_blocks_x, num_blocks_z);

	FilterPointCloudGrid(min_points, num_blocks_x, num_blocks_z);

	AssignCamerasToBlock();

	return;
}

void ViewClustering::BuildPointCloudGrid(const int block_size, const int num_blocks_x, const int num_blocks_z)
{
	for (int i = 0; i < data.points.size(); i++)
	{
		// Horrible thing to fix
		
		const float x = std::floor((data.points[i].x + std::abs(x_min) + 0.01) / block_size);
		const float z = std::floor((data.points[i].z + std::abs(z_min) + 0.01) / block_size);

		const int idx = z * num_blocks_x + x;

		clusters[idx].point_idx.push_back(i);
	}
}

void ViewClustering::FilterPointCloudGrid(const int min_points, const int num_blocks_x, const int num_blocks_z)
{
	// std::cout << "Clusters before filtering: " << clusters.size() << std::endl;

	// Group together small clusters

	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].point_idx.size() < min_points)
		{
			// Compute the size of all neighboring blocks and check if the point is isolated

			// std::cout << "Small cluster with index " << i << std::endl;

			std::vector<int> neighbors = { i - 1, i + 1, i - num_blocks_x, i + num_blocks_x, 
										   i - num_blocks_x - 1, i - num_blocks_x + 1,
										   i + num_blocks_x - 1, i + num_blocks_x + 1 };

			bool isolated = true;
			int smallest_idx = -1;
			for (const auto& idx : neighbors)
			{
				if (idx >= 0 && idx < clusters.size())
				{
					// std::cout << "I can visit the neighbor with index " << idx << std::endl;
					if (clusters[idx].point_idx.size() > 0)
					{
						// std::cout << "The neighbor is not empty" << std::endl;
						isolated = false;
						if (smallest_idx == -1)
						{
							// std::cout << "Initialized smallest neighbor" << std::endl;
							smallest_idx = idx;
						}
						else if (clusters[idx].point_idx.size() < clusters[smallest_idx].point_idx.size())
						{
							// std::cout << "The neighbor is the smallest up to now" << std::endl;
							smallest_idx = idx;
						}
					}
				}
			}

			if (!isolated)
			{
				// std::cout << "The cluster is not isolated" << std::endl;
				clusters[smallest_idx].point_idx.insert(clusters[smallest_idx].point_idx.end(), 
														clusters[i].point_idx.begin(), 
														clusters[i].point_idx.end());
			}
			else
			{
				// std::cout << "The cluster is isolated" << std::endl;
			}

			clusters[i].point_idx.resize(0); // Set size to zero in order to remove later

			// std::cout << "Cluster size: " << clusters[i].point_idx.size() << std::endl;
			// std::cin.get();
		}
	}

	// Remove empty clusters

	const auto lambda_size = [&](const Cluster& c){ return c.point_idx.size() == 0; };
	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), lambda_size), clusters.end());
	
	std::cout << "Clusters after filtering: " << clusters.size() << std::endl;

	int sum = 0;
	for (const auto& c : clusters)
	{
		// std::cout << "Cluster size: " << c.point_idx.size() << std::endl;
		sum += c.point_idx.size();
	}

	std::cout << "Total points in clusters: " << sum << std::endl;
	std::cout << "Total points in input dataset: " << data.points.size() << std::endl;

}

void ViewClustering::AssignCamerasToBlock()
{
	int sum = 0;
	for (auto& c : clusters) // Cluster
	{
		for (const auto& p : c.point_idx) // Points in the cluster 
		{
			for (const auto& f : data.points[p].frame_idx) // Cameras that see points in the cluster
			{
				// Check if the camera is sufficiently close to the point

				const Point p_cam = TransformPointFromWorldToCam(data.images[f].R, data.images[f].t, data.points[p]);
				if (p_cam.z < 20.0f)
				{
					c.camera_idx.push_back(f);
				}
			}
		}

		// Remove duplicates (much more efficient approaches exist)
		std::sort(c.camera_idx.begin(), c.camera_idx.end());
		c.camera_idx.erase(std::unique(c.camera_idx.begin(), c.camera_idx.end()), c.camera_idx.end());
		
		// std::cout << "Cameras in current cluster: " << c.camera_idx.size() << std::endl;
		// std::cout << "Points in current cluster: " << c.point_idx.size() << std::endl;

		sum += c.camera_idx.size();
	}

	std::cout << "Total cameras in clusters: " << sum << std::endl;
	// std::cout << "Total cameras in input dataset: " << data.images.size() << std::endl;
}