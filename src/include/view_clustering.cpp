#include "view_clustering.h"

namespace plt = matplotlibcpp;

void ViewClustering::ClusterViews(const int block_size, const int min_points, const int min_cameras, const float max_distance)
{
	ComputePointCloudRange();

	const int num_blocks_x = std::ceil(std::abs(x_max - x_min) / block_size);
	const int num_blocks_z = std::ceil(std::abs(z_max - z_min) / block_size);
	clusters.resize(num_blocks_x * num_blocks_z);

	AssignPointsToBlock(block_size, num_blocks_x);

	GroupByPoints(min_points, num_blocks_x, num_blocks_z);

	AssignCamerasToBlock(max_distance);

	GroupByCameras(min_cameras, num_blocks_x, num_blocks_z);

	const auto lambda_size = [](const Cluster& c){ return c.point_idx.empty() || c.camera_idx.empty(); };
	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), lambda_size), clusters.end());
}

void ViewClustering::ComputeNeighbors(const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0)
{
	std::vector<std::thread> th_vec;

	for (int i = 0; i < clusters.size(); i++)
	{
		th_vec.push_back(std::thread(&ViewClustering::ComputeNeighborsForCluster, this, i, num_neighbors, sigma_0, sigma_1, theta_0));
	}

	for (auto& th : th_vec)
	{
		th.join();
	}
}

bool ViewClustering::WriteClustersFiles(const std::string& output_path)
{
	if (mkdir(output_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
	{
		std::cout << "Failed to create the results directory" << std::endl;
		return false;
	}

	for (int i = 0; i < clusters.size(); i++)
	{
		const std::string cluster_folder = output_path + "cluster_" + std::to_string(i) + "/";
		if (mkdir(cluster_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
		{
			std::cout << "Failed to create the directory for cluster " << i << std::endl;
			return false;
		}

		if (!WriteCamerasFiles(cluster_folder, i))
		{
			std::cout << "Failed to write cameras files for cluster " << i << std::endl;
			return false;
		}

		if (!WriteNeighborsFile(cluster_folder, i))
		{
			std::cout << "Failed to write neighbors file for cluster " << i << std::endl;
			return false;
		}
	}

	return true;
}

void ViewClustering::PlotClusters()
{
	plt::suptitle("View clustering results");

	plt::subplot(1, 2, 1);
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

	plt::subplot(1, 2, 2);
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

void ViewClustering::ComputePointCloudRange()
{
	x_min = std::numeric_limits<float>::max();
	x_max = -x_min;

	z_min = std::numeric_limits<float>::max();
	z_max = -z_min;

	for (const auto& p : data.points)
	{
		const float x = p.x;
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

void ViewClustering::AssignPointsToBlock(const int block_size, const int num_blocks_x)
{
	for (int i = 0; i < data.points.size(); i++)
	{		
		const float x = std::floor((data.points[i].x + std::abs(x_min) + 0.01) / block_size);
		const float z = std::floor((data.points[i].z + std::abs(z_min) + 0.01) / block_size);
		const int idx = z * num_blocks_x + x;
		clusters[idx].point_idx.push_back(i);
	}
}

void ViewClustering::GroupByPoints(const int min_points, const int num_blocks_x, const int num_blocks_z)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].point_idx.size() > 0 && clusters[i].point_idx.size() < min_points)
		{
			std::vector<int> neighbors = { i - 1, // Left
										   i + 1, // Right
										   i - num_blocks_x, // Up 
										   i + num_blocks_x, // Down
										   i - num_blocks_x - 1, // Up-left 
										   i - num_blocks_x + 1, // Up-right
										   i + num_blocks_x - 1, // Down-left
										   i + num_blocks_x + 1 }; // Down-right

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
}

void ViewClustering::AssignCamerasToBlock(const float max_distance)
{
	for (auto& c : clusters) // Cluster
	{
		for (const auto& p : c.point_idx) // Points in the cluster 
		{
			for (const auto& f : data.points[p].frame_idx) // Cameras that see points in the cluster
			{
				const Point p_cam = TransformPointFromWorldToCam(data.images[f].R, data.images[f].t, data.points[p]);
				if (p_cam.z < max_distance)
				{
					c.camera_idx.push_back(f);
				}
			}
		}

		std::sort(c.camera_idx.begin(), c.camera_idx.end());
		c.camera_idx.erase(std::unique(c.camera_idx.begin(), c.camera_idx.end()), c.camera_idx.end());
	}
}

void ViewClustering::GroupByCameras(const int min_cameras, const int num_blocks_x, const int num_blocks_z)
{
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].camera_idx.size() > 0 && 
			clusters[i].camera_idx.size() < min_cameras && 
			clusters[i].point_idx.size() > 0)
		{
			std::vector<int> neighbors = { i - 1, // Left
										   i + 1, // Right
										   i - num_blocks_x, // Up 
										   i + num_blocks_x, // Down
										   i - num_blocks_x - 1, // Up-left 
										   i - num_blocks_x + 1, // Up-right
										   i + num_blocks_x - 1, // Down-left
										   i + num_blocks_x + 1 }; // Down-right

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
}

void ViewClustering::ComputeNeighborsForCluster(const int i, const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0)
{
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

bool ViewClustering::WriteNeighborsFile(const std::string& path, const int idx)
{
	const std::string filename = path + std::string("pair.txt");

	std::ofstream neighbors_file_stream(filename, std::ios::out);
	if (!neighbors_file_stream)
	{
		std::cout << "Failed to open neighbors file for cluster " << idx << std::endl;
		return false;
	}

	Cluster& c = clusters[idx];

	neighbors_file_stream << c.camera_idx.size() << std::endl;

	for (const auto& i : c.camera_idx)
	{
		neighbors_file_stream << i << std::endl;
		neighbors_file_stream << 10 << " ";
		for (auto& n : c.neighbors[i])
		{
			neighbors_file_stream << n.idx << " " << n.score << " ";
		}
		neighbors_file_stream << std::endl;
	}

	return true;
}

bool ViewClustering::WriteCamerasFiles(const std::string& path, const int idx)
{
	const std::string cam_folder = path + "cams_1/";
	if (mkdir(cam_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
	{
		std::cout << "Failed to create the directory for cluster " << idx << std::endl;
		return false;
	}

	for (const auto& i : clusters[idx].camera_idx)
	{
		char buffer[50];
		sprintf(buffer, "%.8d_cam.txt", i);
		const std::string filename = cam_folder + std::string(buffer);

		std::ofstream cameras_file_stream(filename, std::ios::out);
		if (!cameras_file_stream)
		{
			std::cout << "Failed to open camera file " << i << " for cluster " << idx << std::endl;
			return false;
		}

		cameras_file_stream << "extrinsic" << std::endl;
		cameras_file_stream << data.images[i].R(0,0) << " " << data.images[i].R(0,1) << " " 
							<< data.images[i].R(0,2) << " " << data.images[i].t(0,0) << std::endl
							<< data.images[i].R(1,0) << " " << data.images[i].R(1,1) << " " 
							<< data.images[i].R(1,2) << " " << data.images[i].t(1,0) << std::endl
							<< data.images[i].R(2,0) << " " << data.images[i].R(2,1) << " " 
							<< data.images[i].R(2,2) << " " << data.images[i].t(2,0) << std::endl
							<< 0.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f 
							<< std::endl << std::endl;

		cameras_file_stream << "intrinsic" << std::endl;
		cameras_file_stream << data.images[i].K(0,0) << " " << data.images[i].K(0,1) 
							<< " " << data.images[i].K(0,2) << std::endl
							<< data.images[i].K(1,0) << " " << data.images[i].K(1,1) 
							<< " " << data.images[i].K(1,2) << std::endl
							<< data.images[i].K(2,0) << " " << data.images[i].K(2,1) 
							<< " " << data.images[i].K(2,2) << std::endl << std::endl;

		cameras_file_stream << data.images[i].min_depth << " " << data.images[i].max_depth << std::endl;
	}

	return true;
}