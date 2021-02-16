#include "view_clustering.h"

namespace plt = matplotlibcpp;

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
}

void ViewClustering::ClusterViews(const int block_size, const int min_points, const int min_cameras, const float max_distance)
{
	ComputePointCloudRange();

	const int num_blocks_x = std::ceil(std::abs(x_max - x_min) / block_size);
	const int num_blocks_z = std::ceil(std::abs(z_max - z_min) / block_size);
	clusters.resize(num_blocks_x * num_blocks_z);

	std::cout << "There are " << clusters.size() << " potential clusters" << std::endl;

	AssignPointsToBlock(block_size, num_blocks_x, num_blocks_z);

	GroupByPoints(min_points, num_blocks_x, num_blocks_z);

	AssignCamerasToBlock(max_distance);

	GroupByCameras(min_cameras, num_blocks_x, num_blocks_z);

	const auto lambda_size = [&](const Cluster& c){ return c.point_idx.empty() || c.camera_idx.empty(); };
	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), lambda_size), clusters.end());

	return;
}

void ViewClustering::PlotClusteredPointCloud()
{
	for (const auto& c : clusters)
	{
		std::vector<float> x, z;	
		for (const auto& p : c.point_idx)
		{
			x.push_back(data.points[p].x);
			z.push_back(data.points[p].z);
		}
		plt::plot(x, z, "o ");
	}
	plt::show();
}

void ViewClustering::PlotClusteredTrajectory()
{
	for (const auto& c : clusters)
	{
		std::vector<float> x, z;	
		for (const auto& img : c.camera_idx)
		{
			x.push_back(data.images[img].t(0,0));
			z.push_back(data.images[img].t(2,0));
		}
		plt::plot(x, z, "o ");
	}
	plt::show();
}

void ViewClustering::ComputeNeighbors(const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0)
{	

	for (int i = 0; i < clusters.size(); i++)
	{
		std::cout << "Computing neighbors for cluster " << i << " with " << clusters[i].camera_idx.size() << " cameras" << std::endl;

		for (const auto& ref : clusters[i].camera_idx)
		{
			std::vector<int> idx_ref;
			for (const auto& f : data.images[ref].features)
			{
				idx_ref.push_back(f.point_idx);
			}

			std::vector<Neighbor> n;

			for (const auto& src : clusters[i].camera_idx)
			{
				if (ref != src)
				{
					std::vector<int> idx_comm;
					for (const auto& f : data.images[src].features)
					{
						const int idx_src = f.point_idx;
						if (std::find(idx_ref.begin(), idx_ref.end(), idx_src) != idx_ref.end())
						{
							idx_comm.push_back(idx_src);
						}
					}

					const float score = ComputeViewSelectionScore(idx_comm, ref, src, sigma_0, sigma_1, theta_0);
					if (score > 0.0f)
					{

						n.push_back(Neighbor(src, score));
					}
				}
			}

			const auto lambda_sort = [](const Neighbor& n1, const Neighbor& n2) { return n1.score > n2.score; };
			std::sort(n.begin(), n.end(), lambda_sort);
			n.resize(num_neighbors);

			clusters[i].neighbors.insert(std::make_pair(ref, n));
		}
	}
}

// bool ViewClustering::WriteNeighborsFile(const std::string& path)
// {
// 	const std::string filename = path + std::string("pair.txt");

// 	std::ofstream neighbors_file_stream(filename, std::ios::out);
// 	if (!neighbors_file_stream)
// 	{
// 		std::cout << "Failed to open neighbors file" << std::endl;
// 		return false;
// 	}

// 	neighbors_file_stream << filt.size() << std::endl;

// 	for (const auto& i : filt)
// 	{
// 		std::cout << "Writing neighbors line for image " << i << std::endl;

// 		neighbors_file_stream << i << std::endl;
// 		neighbors_file_stream << 10 << " ";
// 		for (const auto& n : images[i].neighbors)
// 		{
// 			neighbors_file_stream << n.idx << " " << n.score << " ";
// 		}
// 		neighbors_file_stream << std::endl;
// 	}

// 	return true;
// }

void ViewClustering::AssignPointsToBlock(const int block_size, const int num_blocks_x, const int num_blocks_z)
{
	for (int i = 0; i < data.points.size(); i++)
	{
		// Horrible thing to fix
		
		const float x = std::floor((data.points[i].x + std::abs(x_min) + 0.01) / block_size);
		const float z = std::floor((data.points[i].z + std::abs(z_min) + 0.01) / block_size);

		const int idx = z * num_blocks_x + x;

		clusters[idx].point_idx.push_back(i);
	}

	int count = 0;
	for (const auto& c : clusters)
	{
		if (c.point_idx.size() > 0)
		{
			count++;
		}
	}
	std::cout << "There are " << count << " clusters with points assigned" << std::endl;
}

void ViewClustering::GroupByPoints(const int min_points, const int num_blocks_x, const int num_blocks_z)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].point_idx.size() < min_points)
		{
			std::vector<int> neighbors = { i - 1, i + 1, i - num_blocks_x, i + num_blocks_x, 
										   i - num_blocks_x - 1, i - num_blocks_x + 1,
										   i + num_blocks_x - 1, i + num_blocks_x + 1 };

			bool isolated = true;
			int smallest_idx = -1;
			for (const auto& idx : neighbors)
			{
				if (idx >= 0 && idx < clusters.size())
				{
					if (clusters[idx].point_idx.size() > 0)
					{
						isolated = false;
						if (smallest_idx == -1)
						{
							smallest_idx = idx;
						}
						else if (clusters[idx].point_idx.size() < clusters[smallest_idx].point_idx.size())
						{
							smallest_idx = idx;
						}
					}
				}
			}

			if (!isolated)
			{
				clusters[smallest_idx].point_idx.insert(clusters[smallest_idx].point_idx.end(), 
														clusters[i].point_idx.begin(), 
														clusters[i].point_idx.end());
			}

			clusters[i].point_idx.resize(0); // Set size to zero in order to remove later
		}
	}

	int count = 0;
	for (const auto& c : clusters)
	{
		if (c.point_idx.size() > 0)
		{
			count++;
		}
	}
	std::cout << "After grouping by points, there are " << count << " non-empty clusters" << std::endl;
}

void ViewClustering::AssignCamerasToBlock(const float max_distance)
{
	for (auto& c : clusters) // Cluster
	{
		for (const auto& p : c.point_idx) // Points in the cluster 
		{
			for (const auto& f : data.points[p].frame_idx) // Cameras that see points in the cluster
			{
				// Check if the camera is sufficiently close to the point

				const Point p_cam = TransformPointFromWorldToCam(data.images[f].R, data.images[f].t, data.points[p]);
				if (p_cam.z < max_distance)
				{
					c.camera_idx.push_back(f);
				}
			}
		}

		// Remove duplicates (much more efficient approaches exist)
		std::sort(c.camera_idx.begin(), c.camera_idx.end());
		c.camera_idx.erase(std::unique(c.camera_idx.begin(), c.camera_idx.end()), c.camera_idx.end());
	}

	int count = 0;
	for (const auto& c : clusters)
	{
		if (c.camera_idx.size() > 0)
		{
			count++;
		}
	}
	std::cout << "There are " << count << " clusters with cameras assigned" << std::endl;

}

void ViewClustering::GroupByCameras(const int min_cameras, const int num_blocks_x, const int num_blocks_z)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].camera_idx.size() > 0 && clusters[i].camera_idx.size() < min_cameras && clusters[i].point_idx.size() > 0)
		{
			std::vector<int> neighbors = { i - 1, i + 1, i - num_blocks_x, i + num_blocks_x, 
										   i - num_blocks_x - 1, i - num_blocks_x + 1,
										   i + num_blocks_x - 1, i + num_blocks_x + 1 };

			int smallest_idx = -1;
			for (const auto& idx : neighbors)
			{
				if (idx >= 0 && idx < clusters.size())
				{
					if (clusters[idx].point_idx.size() > 0 && clusters[idx].camera_idx.size() > 0)
					{
						if (smallest_idx == -1)
						{
							smallest_idx = idx;
						}
						else if (clusters[idx].camera_idx.size() < clusters[smallest_idx].camera_idx.size())
						{
							smallest_idx = idx;
						}
					}
				}
			}

			if (smallest_idx != -1)
			{
				clusters[smallest_idx].camera_idx.insert(clusters[smallest_idx].camera_idx.end(), 
														 clusters[i].camera_idx.begin(), 
														 clusters[i].camera_idx.end());

				std::sort(clusters[smallest_idx].camera_idx.begin(), clusters[smallest_idx].camera_idx.end());
				clusters[smallest_idx].camera_idx.erase(std::unique(clusters[smallest_idx].camera_idx.begin(), 
																	clusters[smallest_idx].camera_idx.end()),
														clusters[smallest_idx].camera_idx.end());

			}
			else
			{
				std::cout << "The cluster is isolated" << std::endl;
				std::cin.get();
			}

			clusters[i].camera_idx.resize(0); // Set size to zero in order to remove later
		}
	}

	int count = 0;
	for (const auto& c : clusters)
	{
		if (c.camera_idx.size() > 0)
		{
			count++;
		}
	}
	std::cout << "After grouping by cameras, there are " << count << " non-empty clusters" << std::endl;

}

float ViewClustering::ComputeViewSelectionScore(const std::vector<int>& idx, 
												const int ref, 
												const int src,
												const float sigma_0, 
												const float sigma_1, 
												const float theta_0)
{
	float score = 0.0f;
	for (const auto& id : idx)
	{
		float theta = ComputeTriangulationAngle(data.points[id], data.images[ref].t, data.images[src].t);
		if (theta <= theta_0)
		{
			score += std::exp(- std::pow(theta - theta_0, 2) / (2 * std::pow(sigma_0, 2)));
		}
		else
		{
			score += std::exp(- std::pow(theta - theta_0, 2) / (2 * std::pow(sigma_1, 2)));
		}
	}
	return score;
}