#include "io_utils.h"

bool InputDataset::LoadPoints(const std::string& filename)
{
	std::ifstream points_file_stream(filename, std::ios::in);
	if (!points_file_stream)
	{
		std::cout << "Failed to open file " << filename << std::endl;
		return false;
	}
	std::cout << "Opened file " << filename << std::endl;

	size_t num_points;

	std::string line;
	std::getline(points_file_stream, line);
	std::istringstream line_stream(line);

	line_stream >> num_points;
	points.resize(num_points);

	std::cout << "Number of points to load: " << num_points << std::endl;

	while(!points_file_stream.eof() && !points_file_stream.bad())
	{
		std::string line;
		std::getline(points_file_stream, line);
		std::istringstream line_stream(line);

		int point_ID, color, valid;
		Point new_point;
		line_stream >> point_ID 
					>> new_point.x >> new_point.y >> new_point.z 
					>> color >> valid;

		// new_point = TransformPointFromFLUToRDF(new_point);

		points[point_ID] = new_point;
	}

	std::cout << "Loaded " << points.size() << " points" << std::endl << std::endl;

	return true;
}

bool InputDataset::LoadFeatures(const std::string& filename)
{
	std::ifstream features_file_stream(filename, std::ios::in);
	if (!features_file_stream)
	{
		std::cout << "Failed to open file " << filename << std::endl;
		return false;
	}
	std::cout << "Opened file " << filename << std::endl;

	uint64_t buff_64;
	features_file_stream.read((char*) &buff_64, sizeof(uint64_t));
	const int num_features = buff_64;

	std::cout << "Number of features to load: " << num_features << std::endl;

	uint32_t buff;
	features_file_stream.read((char*) &buff, sizeof(uint32_t));
	const int num_frames = buff;
	images.resize(num_frames);

	std::cout << "Number of frames in the sequence: " << num_frames << std::endl;

	features_file_stream.read((char*) &buff, sizeof(uint32_t));
	const int num_observations = buff;

	std::cout << "Number of points in the point cloud: " << num_observations << std::endl;

	int count = 0;
	while (!features_file_stream.eof() && !features_file_stream.bad() && count < num_features)
	{
		Feature new_feature;

		// Point ID
		features_file_stream.read((char*) &buff, sizeof(uint32_t));
		new_feature.point_idx = buff;

		// Color
		features_file_stream.read((char*) &buff, sizeof(uint32_t));

		// Left camera coordinates
		float buff_f;
		features_file_stream.read((char*) &buff_f, sizeof(float));
		new_feature.left.x = buff_f;
		features_file_stream.read((char*) &buff_f, sizeof(float));
		new_feature.left.y = buff_f;

		// Right camera coordinates
		features_file_stream.read((char*) &buff_f, sizeof(float));
		new_feature.right.x = buff_f;
		features_file_stream.read((char*) &buff_f, sizeof(float));
		new_feature.right.y = buff_f;

		// Sensor
		features_file_stream.read((char*) &buff, sizeof(uint32_t));
		const int sensor = buff;

		// Frame
		features_file_stream.read((char*) &buff, sizeof(uint32_t));
		const int frame = buff;

		if (sensor == 0)
		{
			images[frame].features.push_back(new_feature);
		}

		count++;
	}

	std::cout << "Loaded " << count << " features" << std::endl << std::endl;

	return true;
}

bool InputDataset::LoadPoses(const std::string& filename)
{
	std::ifstream poses_file_stream(filename, std::ios::in);
	if (!poses_file_stream)
	{
		std::cout << "Failed to open file " << filename << std::endl;
		return false;
	}
	std::cout << "Opened file " << filename << std::endl;

	int count = 0;
	while(!poses_file_stream.eof() && !poses_file_stream.bad() && count < images.size())
	{
		std::string line;
		std::getline(poses_file_stream, line);
		std::istringstream line_stream(line);

		images[count].K = cv::Mat::eye(3, 3, CV_32F);
		images[count].R = cv::Mat::eye(3, 3, CV_32F);
		images[count].t = cv::Mat::eye(3, 1, CV_32F);

		images[count].width = 3840;
		images[count].height = 1920;
		images[count].baseline = 0.3003000020980835;
		images[count].K << 2654.375, 0, 1834.875,
						   0, 2654.375, 978.625,
						   0, 0, 1;

		line_stream >> images[count].R(0,0) >> images[count].R(0,1) >> images[count].R(0,2) >> images[count].t(0,0)
					>> images[count].R(1,0) >> images[count].R(1,1) >> images[count].R(1,2) >> images[count].t(1,0)
					>> images[count].R(2,0) >> images[count].R(2,1) >> images[count].R(2,2) >> images[count].t(2,0);

		char buffer[50];
		sprintf(buffer, "%.6d.png", count);
		images[count].filename = std::string(buffer);

		count++;
	}	

	std::cout << "Loaded " << count << " poses" << std::endl << std::endl;
	return true;
}

std::vector<int> InputDataset::FilterPoses(const float min_dist)
{
	int prev = 0;
	std::vector<int> filtered_poses;
	filtered_poses.push_back(prev); 

	for (int i = 1; i < images.size(); i++)
	{
		const float dist = ComputePoseDistance(images[prev].t, images[i].t);
		if (dist > min_dist)
		{
			prev = i;
			filtered_poses.push_back(prev);
		}
	} 

	std::cout << "Filtered " << filtered_poses.size() << " poses" << std::endl;
	return filtered_poses;
}

void InputDataset::BuildFeatureTracks(const std::vector<int>& filt)
{
	for (const auto& i : filt)
	{
		for (const auto& f : images[i].features)
		{
			points[f.point_idx].frame_idx.push_back(i);
		}
	}
}

void InputDataset::ComputeDepthRange(const std::vector<int>& filt)
{
	for (const auto& i : filt)
	{
		std::cout << "Computing depth range for image " << i << std::endl;

		float max_depth = 0.0f;
		float min_depth = std::numeric_limits<float>::max();

		for (const auto& f : images[i].features)
		{
			const int idx = f.point_idx;
			const Point p_cam = TransformPointFromWorldToCam(images[i].R, images[i].t, points[idx]);
			const float depth = p_cam.z;
			if (depth < min_depth)
			{
				min_depth = depth;
			}
			if (depth > max_depth)
			{
				max_depth = depth;
			}
		}

		images[i].min_depth = min_depth;
		images[i].max_depth = max_depth;

		std::cout << "Computed range: (" << min_depth << ", " << max_depth << ")" << std::endl;
	}
}

void InputDataset::ComputeNeighbors(const std::vector<int>& filt, 
									const float sigma_0, 
									const float sigma_1, 
									const float theta_0)
{
	for (const auto& ref : filt)
	{
		std::cout << "Computing neighbors for image " << ref << std::endl;

		std::vector<int> idx_ref;
		for (const auto& f : images[ref].features)
		{
			idx_ref.push_back(f.point_idx);
		}

		for (const auto& src : filt)
		{
			if (ref != src)
			{
				std::vector<int> idx_comm;
				for (const auto& f : images[src].features)
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
					images[ref].neighbors.push_back(Neighbor(src, score));
					// std::cout << "Image " << ref << " and image " << src << " have " << idx_comm.size() 
					// 		  << " features in common and the score is " << score << std::endl;
					// std::cin.get();
				}
			}
		}

		const auto lambda_sort = [](const Neighbor& n1, const Neighbor& n2) { return n1.score > n2.score; };
		std::sort(images[ref].neighbors.begin(), images[ref].neighbors.end(), lambda_sort);
		images[ref].neighbors.resize(10);

		// std::cout << "Neighbors found for image " << ref << " are:" << std::endl;
		// for (const auto& n : images[ref].neighbors)
		// {
		// 	std::cout << "Image " << n.idx << " with score " << n.score << std::endl;
		// }
		// std::cin.get();
	}
}

bool InputDataset::WriteCameraFiles(const std::vector<int>& filt, const std::string& path)
{
	for (const auto& i : filt)
	{
		std::cout << "Writing camera file for image " << i << std::endl;

		char buffer[50];
		sprintf(buffer, "%.8d_cam.txt", i);
		const std::string filename = path + std::string(buffer);

		std::ofstream cameras_file_stream(filename, std::ios::out);
		if (!cameras_file_stream)
		{
			std::cout << "Failed to open camera file " << i << std::endl;
			return false;
		}

		cameras_file_stream << "extrinsic" << std::endl;
		cameras_file_stream << images[i].R(0,0) << " " << images[i].R(0,1) << " " 
							<< images[i].R(0,2) << " " << images[i].t(0,0) << std::endl
							<< images[i].R(1,0) << " " << images[i].R(1,1) << " " 
							<< images[i].R(1,2) << " " << images[i].t(1,0) << std::endl
							<< images[i].R(2,0) << " " << images[i].R(2,1) << " " 
							<< images[i].R(2,2) << " " << images[i].t(2,0) << std::endl
							<< 0.0f << " " << 0.0f << " " << 0.0f << " " << 1.0f 
							<< std::endl << std::endl;

		cameras_file_stream << "intrinsic" << std::endl;
		cameras_file_stream << images[i].K(0,0) << " " << images[i].K(0,1) 
							<< " " << images[i].K(0,2) << std::endl
							<< images[i].K(1,0) << " " << images[i].K(1,1) 
							<< " " << images[i].K(1,2) << std::endl
							<< images[i].K(2,0) << " " << images[i].K(2,1) 
							<< " " << images[i].K(2,2) << std::endl << std::endl;

		cameras_file_stream << images[i].min_depth << " " << images[i].max_depth << std::endl;

		// std::cin.get();
	}

	return true;
}

bool InputDataset::WriteNeighborsFile(const std::vector<int>& filt, const std::string& path)
{
	const std::string filename = path + std::string("pair.txt");

	std::ofstream neighbors_file_stream(filename, std::ios::out);
	if (!neighbors_file_stream)
	{
		std::cout << "Failed to open neighbors file" << std::endl;
		return false;
	}

	neighbors_file_stream << filt.size() << std::endl;

	for (const auto& i : filt)
	{
		std::cout << "Writing neighbors line for image " << i << std::endl;

		neighbors_file_stream << i << std::endl;
		neighbors_file_stream << 10 << " ";
		for (const auto& n : images[i].neighbors)
		{
			neighbors_file_stream << n.idx << " " << n.score << " ";
		}
		neighbors_file_stream << std::endl;
	}

	return true;
}

float InputDataset::ComputeViewSelectionScore(const std::vector<int>& idx, 
											  const int ref, 
											  const int src,
											  const float sigma_0, 
											  const float sigma_1, 
											  const float theta_0)
{
	float score = 0.0f;
	for (const auto& id : idx)
	{
		float theta = ComputeTriangulationAngle(points[id], images[ref].t, images[src].t);
		// std::cout << "Triangulation angle: " << theta << std::endl;
		// std::cin.get();
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