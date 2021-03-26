#include "clustering.h"

Clustering::Clustering(const Parameters& p, InputDataset& d) : params(p), data(d) {};

void Clustering::ComputeClusters()
{
	// Step 1: process points

	ComputePointCloudRange();

	AssignPointsToBlock();
	std::cout << "Assigned points to each block." << std::endl;
	CountNonEmptyClusters();

	GroupByPoints();
	std::cout << "Grouped clusters by points." << std::endl;
	CountNonEmptyClusters();

	// Step 2: process cameras

	AssignCamerasToBlock();
	GroupByCameras();
	std::cout << "Assigned and grouped cameras for each block." << std::endl;
	CountNonEmptyClusters();

	// Step 3: locally optimize each cluster (TODO)

	// PrintReport();
}

void Clustering::ComputePointCloudRange()
{
	// std::vector<double> x, z;
	// for (const auto& p : data.points)
	// {
	// 	x.push_back(p.x);
	// 	z.push_back(p.z);
	// }

	// std::sort(x.begin(), x.end());
	// std::sort(z.begin(), z.end());

	// const int low_idx = static_cast<int>(0.01 * x.size());
	// const int high_idx = static_cast<int>(0.99 * x.size());

	// x_min = x[low_idx];
	// x_max = x[high_idx];
	// z_min = z[low_idx];
	// z_max = z[high_idx];

	const auto cmp_x = [](const Point& p1, const Point& p2) { return p1.x < p2.x; }; 
	const auto minmax_x = std::minmax_element(data.points.begin(), data.points.end(), cmp_x);
	x_min = minmax_x.first->x;
	x_max = minmax_x.second->x;

	const auto cmp_z = [](const Point& p1, const Point& p2) { return p1.z < p2.z; }; 
	const auto minmax_z = std::minmax_element(data.points.begin(), data.points.end(), cmp_z);
	z_min = minmax_z.first->z;
	z_max = minmax_z.second->z;

	num_blocks_x = std::ceil(std::abs(x_max - x_min) / params.block_size);
	num_blocks_z = std::ceil(std::abs(z_max - z_min) / params.block_size);
	clusters.resize(num_blocks_x * num_blocks_z);

	std::cout << "Point cloud range on x: (" << x_min << ", " << x_max << ")" << std::endl;
	std::cout << "Point cloud range on z: (" << z_min << ", " << z_max << ")" << std::endl;
	std::cout << "There are " << clusters.size() << " potential clusters of size " << params.block_size << " m" << std::endl;
}

void Clustering::AssignPointsToBlock()
{
	for (int i = 0; i < data.points.size(); i++)
	{		
		const float x = std::floor((data.points[i].x + std::abs(x_min) + 0.01) / params.block_size);
		const float z = std::floor((data.points[i].z + std::abs(z_min) + 0.01) / params.block_size);
		const int idx = z * num_blocks_x + x;
		clusters[idx].point_idx.push_back(i);
	}
}

void Clustering::GroupByPoints()
{
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].point_idx.size() > 0 && // Otherwise, just skip
			clusters[i].point_idx.size() < params.min_points)
		{
			std::vector<int> neighbors = { i - num_blocks_x, // Up 
										   i + num_blocks_x, // Down
										   i - num_blocks_x - 1, // Up-left 
										   i - num_blocks_x + 1, // Up-right
										   i + num_blocks_x - 1, // Down-left
										   i + num_blocks_x + 1 }; // Down-right

			if (i % num_blocks_x != 0) // If the block is not on the left border
			{
				neighbors.push_back(i - 1);
			}
			if (i % num_blocks_x != num_blocks_x - 1) // If the block is not on the right border
			{
				neighbors.push_back(i + 1);
			}

			bool isolated = true;
			int smallest_idx = -1;
			for (const auto& idx : neighbors) // Check all its neighbors
			{
				if (idx >= 0 && idx < clusters.size()) // If the current neighbor is valid
				{
					if (clusters[idx].point_idx.size() > 0) // If the current neighbor is not empty
					{
						isolated = false; // If the neighbor is valid and non-empty, the cluster is not isolated
						if (smallest_idx == -1)
						{
							smallest_idx = idx;
						}
						else if (clusters[idx].point_idx.size() < clusters[smallest_idx].point_idx.size())
						{
							smallest_idx = idx; // We are looking for the smallest cluster among neighbors 
						}
					}
				}
			}

			if (!isolated) // If the cluster is isolated, just remove it. Otherwise copy the points
			{
				clusters[smallest_idx].point_idx.insert(clusters[smallest_idx].point_idx.end(), 
														clusters[i].point_idx.begin(), 
														clusters[i].point_idx.end());
			}

			clusters[i].point_idx = {}; // Set size to zero in order to remove later
		}
	}
}

void Clustering::AssignCamerasToBlock()
{
	for (auto& c : clusters) // Cluster
	{
		for (const auto& p : c.point_idx) // Points in the cluster 
		{
			for (const auto& cam : data.points[p].image_idx) // Cameras that see the point
			{
				const int uuid = cam.first;
				const int sensor = uuid / data.num_frames;
				const int frame = uuid - sensor * data.num_frames;

				c.camera_idx.insert(uuid);

				// const Point p_cam = TransformPointFromWorldToCam(data.images[frame][sensor].R, 
				// 												 data.images[frame][sensor].t, 
				// 												 data.points[p]);
				// if (p_cam.z < max_distance)
				// {
				// 	c.camera_idx.insert(uuid); // This should remove duplicates automatically
				// }
			}
		}

		// Remove cameras that see only a few points

		for (auto it = c.camera_idx.begin(); it != c.camera_idx.end();)
		{
			const int uuid = *it;
			const int sensor = uuid / data.num_frames;
			const int frame = uuid - sensor * data.num_frames;

			std::vector<int> curr_idx;
			for (const auto& f : data.images[frame][sensor].features)
			{
				curr_idx.push_back(f.point_idx);
			}

			std::sort(curr_idx.begin(), curr_idx.end());
			std::sort(c.point_idx.begin(), c.point_idx.end());

			std::vector<int> common_idx;
			std::set_intersection(curr_idx.begin(), curr_idx.end(), c.point_idx.begin(), c.point_idx.end(), std::back_inserter(common_idx));

			if (common_idx.size() < 10) // Move to parameters
			{
				it = c.camera_idx.erase(it);
			}
			else
			{
				++it;
			}
		}
	}
}

void Clustering::GroupByCameras()
{
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].camera_idx.size() > 0 && // If the cluster is not empty
			clusters[i].camera_idx.size() < params.min_cameras && // If the cluster has only a few cameras
			clusters[i].point_idx.size() > 0) // Just a sanity check, can be removed
		{
			std::vector<int> neighbors = { i - num_blocks_x, // Up 
										   i + num_blocks_x, // Down
										   i - num_blocks_x - 1, // Up-left 
										   i - num_blocks_x + 1, // Up-right
										   i + num_blocks_x - 1, // Down-left
										   i + num_blocks_x + 1 }; // Down-right

			if (i % num_blocks_x != 0) // If the block is not on the left border
				neighbors.push_back(i - 1);
			if (i % num_blocks_x != num_blocks_x - 1) // If the block is not on the right border
				neighbors.push_back(i + 1);

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

			if (smallest_idx != -1) // If the cluster is isolated, just remove it. Otherwise copy the points
			{
				for (const auto& cam : clusters[i].camera_idx)
				{
					clusters[smallest_idx].camera_idx.insert(cam);
				}

				clusters[smallest_idx].point_idx.insert(clusters[smallest_idx].point_idx.end(), 
														clusters[i].point_idx.begin(), 
														clusters[i].point_idx.end());
			}

			clusters[i].point_idx = {};
			clusters[i].camera_idx.clear(); // Set size to zero in order to remove later
		}
	}
}

void Clustering::PrintReport()
{
	std::cout << std::endl;

	int cam_count = 0;
	int point_count = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].point_idx.size() > 0)
		{
			const int num_points = clusters[i].point_idx.size();
			const int num_cameras = clusters[i].camera_idx.size();
			std::cout << "Cluster " << i << " has " << num_points << " points and " << num_cameras << " cameras" << std::endl;
			cam_count += num_cameras;
			point_count += num_points;
		}
	}
	std::cout << std::endl << "Total points: " << point_count << std::endl;
	std::cout << "Total cameras: " << cam_count << std::endl;

	const float ratio = cam_count / static_cast<float>(params.num_cameras * data.idx_filt.size());
	std::cout << "Cameras ratio: " << ratio << std::endl << std::endl;
}

int Clustering::CountNonEmptyClusters()
{
	int count = 0;
	for (const auto& c : clusters)
	{
		if (!c.point_idx.empty())
		{
			count++;
		}
	}
	std::cout << "There are " << count << " non-empty clusters." << std::endl;
	return count;
}

// void Clustering::ComputeNeighbors()
// {
// 	std::vector<std::thread> th_vec;

// 	for (int i = 0; i < clusters.size(); i++)
// 	{
// 		for (const auto& uuid : clusters[i].camera_idx)
// 		{
// 			const int sensor = uuid / data.num_frames;
// 			const int frame = uuid - sensor * data.num_frames;
			
// 			std::sort(data.images[frame][sensor].features.begin(),
// 					  data.images[frame][sensor].features.end());
// 		}
// 		th_vec.push_back(std::thread(&Clustering::ComputeNeighborsForCluster, this, i));
// 	}

// 	for (auto& th : th_vec)
// 	{
// 		th.join();
// 	}
// }

// void Clustering::ComputeNeighborsForCluster(const int i)
// {
// 	for (const auto& ref : clusters[i].camera_idx)
// 	{
// 		const int ref_sensor = ref / data.num_frames;
// 		const int ref_frame = ref - ref_sensor * data.num_frames;
// 		std::vector<Neighbor> n;

// 		for (const auto& src : clusters[i].camera_idx)
// 		{
// 			if (ref != src)
// 			{
// 				const int src_sensor = src / data.num_frames;
// 				const int src_frame = src - src_sensor * data.num_frames;
// 				std::vector<Feature> common_features;
// 				std::set_intersection(data.images[ref_frame][ref_sensor].features.begin(), data.images[ref_frame][ref_sensor].features.end(),
// 									  data.images[src_frame][src_sensor].features.begin(), data.images[src_frame][src_sensor].features.end(),
// 									  back_inserter(common_features));

// 				const float score = ComputeViewSelectionScore(common_features, ref_frame, ref_sensor, src_frame, src_sensor, sigma_0, sigma_1, theta_0);
// 				if (score > 0.0f)
// 				{
// 					n.push_back(Neighbor(src, score));
// 				}
// 			}
// 		}

// 		const auto lambda_sort = [](const Neighbor& n1, const Neighbor& n2) { return n1.score > n2.score; };
// 		std::sort(n.begin(), n.end(), lambda_sort);
// 		n.resize(num_neighbors);

// 		clusters[i].neighbors.insert(std::make_pair(ref, n));
// 	}
// }

// float Clustering::ComputeViewSelectionScore(const std::vector<Feature>& idx, 
// 												const int ref_frame,
// 												const int ref_sensor,
// 												const int src_frame,
// 												const int src_sensor)
// {
// 	float score = 0.0f;
// 	for (const auto& f : idx)
// 	{
// 		float theta = ComputeTriangulationAngle(data.points[f.point_idx], 
// 												data.images[ref_frame][ref_sensor].t, 
// 												data.images[src_frame][src_sensor].t);
// 		if (theta <= theta_0)
// 		{
// 			score += std::exp(- std::pow(theta - theta_0, 2) / (2 * std::pow(sigma_0, 2)));
// 		}
// 		else
// 		{
// 			score += std::exp(- std::pow(theta - theta_0, 2) / (2 * std::pow(sigma_1, 2)));
// 		}
// 	}
// 	return score;
// }