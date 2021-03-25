#include "clustering.h"

Clustering::Clustering(const Parameters& p, InputDataset& d) : params(p), data(d) {};

void Clustering::ComputeClusters()
{
	// Step 1: process points

	ComputePointCloudRange();
	AssignPointsToBlock();
	GroupByPoints();

	// // Step 2: process cameras

	// AssignCamerasToBlock(max_distance);
	// GroupByCameras(min_cameras, num_blocks_x);

	// // Remove empty clusters

	// const auto lambda_size = [](const Cluster& c){ return c.point_idx.empty() || c.camera_idx.empty(); };
	// clusters.erase(std::remove_if(clusters.begin(), clusters.end(), lambda_size), clusters.end());

	PrintReport();
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

	int count = 0;
	for (const auto& c : clusters)
	{
		if (!c.point_idx.empty())
		{
			count++;
		}
	}
	std::cout << "There are " << count << " non-empty clusters" << std::endl;
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

	int count = 0;
	std::ofstream clusters_file_stream("/home/c-morsingher/clusters.txt", std::ios::out);
	for (const auto& c : clusters)
	{
		if (!c.point_idx.empty())
		{
			for (const auto& idx : c.point_idx)
			{
				clusters_file_stream << data.points[idx].x << " " << data.points[idx].z << " ";
			}
			clusters_file_stream << std::endl;
			count++;
		}
	}
	std::cout << "After grouping, there are " << count << " non-empty clusters" << std::endl;
}

void Clustering::PrintReport()
{
	std::cout << std::endl;
	
	// int cam_count = 0;
	int point_count = 0;
	for (int i = 0; i < clusters.size(); i++)
	{
		if (clusters[i].point_idx.size() > 0)
		{
			const int num_points = clusters[i].point_idx.size();
			// const int num_cams = clusters[i].camera_idx.size();
			std::cout << "Cluster " << i << " has " << num_points << " points" << std::endl;
			// cam_count += num_cams;
			point_count += num_points;
		}
	}
	std::cout << std::endl << "Total points: " << point_count << std::endl << std::endl;
	// std::cout << "Total cameras: " << cam_count << std::endl << std::endl;
}

// void Clustering::ComputeNeighbors(const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0)
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
// 		th_vec.push_back(std::thread(&Clustering::ComputeNeighborsForCluster, this, i, num_neighbors, sigma_0, sigma_1, theta_0));
// 	}

// 	for (auto& th : th_vec)
// 	{
// 		th.join();
// 	}
// }

// bool Clustering::WriteColmapFiles(const std::string& output_path)
// {
// 	for (int i = 0; i < clusters.size(); i++)
// 	{
// 		const std::string cluster_folder = output_path + "cluster_" + std::to_string(i) + "/";
// 		if (mkdir(cluster_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
// 		{
// 			std::cout << "Failed to create the directory for cluster " << i << std::endl;
// 			return false;
// 		}

// 		const std::string colmap_path = cluster_folder + "COLMAP/";
// 		if (mkdir(colmap_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
// 		{
// 			std::cout << "Failed to create the COLMAP directory for cluster " << i << std::endl;
// 			return false;
// 		}

// 		// std::cout << "Writing cameras file in COLMAP format..." << std::endl;
// 		if (!WriteColmapCamerasFile(colmap_path, i))
// 		{
// 			std::cout << "Failed to write cameras file in COLMAP format for cluster " << i << std::endl;
// 			return false;
// 		}

// 		// std::cout << "Writing images file in COLMAP format..." << std::endl;
// 		if (!WriteColmapImagesFile(colmap_path, i))
// 		{
// 			std::cout << "Failed to write images file in COLMAP format for cluster " << i << std::endl;
// 			return false;
// 		}

// 		// std::cout << "Writing points file in COLMAP format..." << std::endl;
// 		if (!WriteColmapPointsFile(colmap_path, i))
// 		{
// 			std::cout << "Failed to write points file in COLMAP format for cluster " << i << std::endl;
// 			return false;
// 		}
// 	}

// 	return true;
// }

// bool Clustering::WriteClustersFiles(const std::string& output_path, const int num_neighbors)
// {
// 	for (int i = 0; i < clusters.size(); i++)
// 	{
// 		const std::string cluster_folder = output_path + "cluster_" + std::to_string(i) + "/";

// 		if (!WriteCamerasFiles(cluster_folder, i))
// 		{
// 			std::cout << "Failed to write cameras files for cluster " << i << std::endl;
// 			return false;
// 		}

// 		if (!WriteNeighborsFile(cluster_folder, i, num_neighbors))
// 		{
// 			std::cout << "Failed to write neighbors file for cluster " << i << std::endl;
// 			return false;
// 		}

// 		if (!WriteImages(cluster_folder, i))
// 		{
// 			std::cout << "Failed to write images for cluster " << i << std::endl;
// 			return false;
// 		}
// 	}

// 	return true;
// }

// void Clustering::AssignCamerasToBlock(const float max_distance)
// {
// 	for (auto& c : clusters) // Cluster
// 	{
// 		for (const auto& p : c.point_idx) // Points in the cluster 
// 		{
// 			for (const auto& cam : data.points[p].image_idx) // Cameras that see the point
// 			{
// 				const int uuid = cam.first;
// 				const int sensor = uuid / data.num_frames;
// 				const int frame = uuid - sensor * data.num_frames;

// 				const Point p_cam = TransformPointFromWorldToCam(data.images[frame][sensor].R, 
// 																 data.images[frame][sensor].t, 
// 																 data.points[p]);
// 				if (p_cam.z < max_distance)
// 				{
// 					c.camera_idx.insert(uuid); // This should remove duplicates automatically
// 				}
// 			}
// 		}
// 	}
// }

// void Clustering::GroupByCameras(const int min_cameras, const int num_blocks_x)
// {
// 	for (int i = 0; i < clusters.size(); i++)
// 	{
// 		if (clusters[i].camera_idx.size() > 0 && 
// 			clusters[i].camera_idx.size() < min_cameras && 
// 			clusters[i].point_idx.size() > 0)
// 		{
// 			std::vector<int> neighbors = { i - num_blocks_x, // Up 
// 										   i + num_blocks_x, // Down
// 										   i - num_blocks_x - 1, // Up-left 
// 										   i - num_blocks_x + 1, // Up-right
// 										   i + num_blocks_x - 1, // Down-left
// 										   i + num_blocks_x + 1 }; // Down-right

// 			if (i % num_blocks_x != 0)
// 				neighbors.push_back(i - 1);
// 			if (i % num_blocks_x != num_blocks_x - 1)
// 				neighbors.push_back(i + 1);

// 			int smallest_idx = -1;
// 			for (const auto& idx : neighbors)
// 			{
// 				if (idx >= 0 && idx < clusters.size())
// 				{
// 					if (clusters[idx].point_idx.size() > 0 && clusters[idx].camera_idx.size() > 0)
// 					{
// 						if (smallest_idx == -1)
// 						{
// 							smallest_idx = idx;
// 						}
// 						else if (clusters[idx].camera_idx.size() < clusters[smallest_idx].camera_idx.size())
// 						{
// 							smallest_idx = idx;
// 						}
// 					}
// 				}
// 			}

// 			if (smallest_idx != -1)
// 			{
// 				for (const auto& cam : clusters[i].camera_idx)
// 				{
// 					clusters[smallest_idx].camera_idx.insert(cam);
// 				}
// 			}
// 			// else
// 			// {
// 			// 	std::cout << "The cluster is isolated" << std::endl;
// 			// 	std::cin.get();
// 			// }

// 			clusters[i].camera_idx.clear(); // Set size to zero in order to remove later
// 		}
// 	}
// }

// void Clustering::ComputeNeighborsForCluster(const int i, const int num_neighbors, const float sigma_0, const float sigma_1, const float theta_0)
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
// 												const int src_sensor,
// 												const float sigma_0, 
// 												const float sigma_1, 
// 												const float theta_0)
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

// bool Clustering::WriteNeighborsFile(const std::string& path, const int idx, const int num_neighbors)
// {
// 	const std::string filename = path + std::string("neighbors.txt");

// 	std::ofstream neighbors_file_stream(filename, std::ios::out);
// 	if (!neighbors_file_stream)
// 	{
// 		std::cout << "Failed to open neighbors file for cluster " << idx << std::endl;
// 		return false;
// 	}

// 	Cluster& c = clusters[idx];

// 	neighbors_file_stream << c.camera_idx.size() << std::endl;

// 	for (const auto& i : c.camera_idx)
// 	{
// 		neighbors_file_stream << i << std::endl;
// 		neighbors_file_stream << c.neighbors[i].size() << " ";
// 		for (auto& n : c.neighbors[i])
// 		{
// 			neighbors_file_stream << n.uuid << " " << n.score << " ";
// 		}
// 		neighbors_file_stream << std::endl;
// 	}

// 	return true;
// }

// bool Clustering::WriteCamerasFiles(const std::string& path, const int idx)
// {
// 	const std::string cam_folder = path + "cameras/";
// 	if (mkdir(cam_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
// 	{
// 		std::cout << "Failed to create the cameras directory for cluster " << idx << std::endl;
// 		return false;
// 	}

// 	for (const auto& uuid : clusters[idx].camera_idx)
// 	{
// 		const int sensor = uuid / data.num_frames;
// 		const int frame = uuid - sensor * data.num_frames;

// 		char buffer[50];
// 		sprintf(buffer, "%.8d.txt", uuid);
// 		const std::string filename = cam_folder + std::string(buffer);

// 		std::ofstream cameras_file_stream(filename, std::ios::out);
// 		if (!cameras_file_stream)
// 		{
// 			std::cout << "Failed to open camera file " << uuid << " for cluster " << idx << std::endl;
// 			return false;
// 		}

// 		cameras_file_stream << "extrinsic" << std::endl;
// 		cameras_file_stream << data.images[frame][sensor].R(0,0) << " " << data.images[frame][sensor].R(0,1) << " " 
// 							<< data.images[frame][sensor].R(0,2) << " " << data.images[frame][sensor].t(0,0) << std::endl
// 							<< data.images[frame][sensor].R(1,0) << " " << data.images[frame][sensor].R(1,1) << " " 
// 							<< data.images[frame][sensor].R(1,2) << " " << data.images[frame][sensor].t(1,0) << std::endl
// 							<< data.images[frame][sensor].R(2,0) << " " << data.images[frame][sensor].R(2,1) << " " 
// 							<< data.images[frame][sensor].R(2,2) << " " << data.images[frame][sensor].t(2,0) << std::endl
// 							<< 0.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f << " "
// 							<< std::endl << std::endl;

// 		cameras_file_stream << "intrinsic" << std::endl;
// 		cameras_file_stream << data.images[frame][sensor].K(0,0) << " " << data.images[frame][sensor].K(0,1) 
// 							<< " " << data.images[frame][sensor].K(0,2) << " " << std::endl
// 							<< data.images[frame][sensor].K(1,0) << " " << data.images[frame][sensor].K(1,1) 
// 							<< " " << data.images[frame][sensor].K(1,2) << " " << std::endl
// 							<< data.images[frame][sensor].K(2,0) << " " << data.images[frame][sensor].K(2,1) 
// 							<< " " << data.images[frame][sensor].K(2,2) << " " << std::endl << std::endl;

// 		cameras_file_stream << data.images[frame][sensor].min_depth << " " 
// 							<< data.images[frame][sensor].max_depth << " " << std::endl << std::endl;

// 		cameras_file_stream << data.images[frame][sensor].filename << " " << std::endl;
// 	}

// 	return true;
// }

// bool Clustering::WriteColmapCamerasFile(const std::string& path, const int idx)
// {
// 	const std::string filename = path + "cameras.txt";
// 	std::ofstream cameras_file_stream(filename, std::ios::out);
// 	if (!cameras_file_stream)
// 	{
// 		std::cout << "Failed to open cameras file for cluster " << idx << std::endl;
// 		return false;
// 	}

// 	cameras_file_stream << "# List of cameras " << std::endl;
	
// 	for (int i = 0; i < data.num_cameras; i++)
// 	{
// 		cameras_file_stream << i << " PINHOLE " << data.images[0][i].width << " " << data.images[0][i].height
// 							<< " " << data.images[0][i].K(0,0) << " " << data.images[0][i].K(1,1)
// 							<< " " << data.images[0][i].K(0,2) << " " << data.images[0][i].K(1,2)
// 							<< " " << std::endl;
// 	}

// 	return true;
// }

// bool Clustering::WriteColmapImagesFile(const std::string& path, const int idx)
// {
// 	const std::string filename = path + "images.txt";
// 	std::ofstream images_file_stream(filename, std::ios::out);
// 	if (!images_file_stream)
// 	{
// 		std::cout << "Failed to open images file for cluster " << idx << std::endl;
// 		return false;
// 	}
	
// 	images_file_stream << "# List of images " << std::endl;

// 	for (const auto& uuid : clusters[idx].camera_idx)
// 	{
// 		const int sensor = uuid / data.num_frames;
// 		const int frame = uuid - sensor * data.num_frames;

// 		Quaternion q = QuaternionFromRotationMatrix(data.images[frame][sensor].R);
// 		const cv::Mat_<float>& t = data.images[frame][sensor].t;

// 		images_file_stream << uuid << " " 
// 						   << q[0] << " " << q[1] << " " << q[2] << " "<< q[3] << " "
// 						   << t(0,0) << " " << t(0,1) << " " << t(0,2) << " "
// 						   << sensor << " " << data.images[frame][sensor].filename << " "
// 						   << std::endl;

// 		for (const auto& f : data.images[frame][sensor].features)
// 		{
// 			images_file_stream << f.right.x << " " << f.right.y << " " << f.point_idx << " "; 
// 		}

// 		images_file_stream << std::endl;
// 	}

// 	return true;
// }

// bool Clustering::WriteColmapPointsFile(const std::string& path, const int idx)
// {
// 	const std::string filename = path + "points3D.txt";
// 	std::ofstream points_file_stream(filename, std::ios::out);
// 	if (!points_file_stream)
// 	{
// 		std::cout << "Failed to open points file for cluster " << idx << std::endl;
// 		return false;
// 	}
	
// 	points_file_stream << "# List of points " << std::endl;

// 	for (const auto& p : clusters[idx].point_idx)
// 	{
// 		points_file_stream << p << " "
// 						   << data.points[p].x << " " << data.points[p].y << " " << data.points[p].z << " "
// 						   << data.points[p].r << " " << data.points[p].g << " " << data.points[p].b << " "
// 						   << data.points[p].error << " ";

// 		for (const auto& f : data.points[p].image_idx)
// 		{
// 			points_file_stream << f.first << " " << f.second << " ";
// 		}

// 		points_file_stream << std::endl;
// 	}

// 	return true;
// }

// // DISCLAIMER: VERY STUPID FUNCTION, A LOT OF THINGS SHOULD BE FIXED LIKE RESCALING OF CAMERAS,
// // RESCALING OF FEATURES AND SO ON

// bool Clustering::WriteImages(const std::string& path, const int idx)
// {
// 	const std::string img_folder = path + "images/";
// 	if (mkdir(img_folder.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH) < 0)
// 	{
// 		std::cout << "Failed to create the images directory for cluster " << idx << std::endl;
// 		return false;
// 	}

// 	int count = 0;
// 	for (const auto& uuid : clusters[idx].camera_idx)
// 	{
// 		const int sensor = uuid / data.num_frames;
// 		const int frame = uuid - sensor * data.num_frames;

// 		const std::string input_path = "/home/c-morsingher/datasets/vislab/full_sequence/full_size/FC/";
// 		const std::string filename = input_path + data.images[frame][sensor].filename;

// 		cv::Mat img = cv::imread(filename, cv::IMREAD_COLOR);
// 		if (img.empty())
// 		{
// 			std::cout << "Failed to load image " << filename << std::endl;
// 			return false;
// 		}

// 		cv::resize(img, img, cv::Size(), 0.25, 0.25);
		
// 		char buffer[50];
// 		sprintf(buffer, "%.8d.jpg", count);
// 		imwrite(img_folder + std::string(buffer), img);
// 		count++;
// 	}

// 	return true;
// }