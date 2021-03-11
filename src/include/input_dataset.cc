#include "input_dataset.h"

bool InputDataset::LoadPoints(const std::string& filename)
{
	std::ifstream points_file_stream(filename, std::ios::in);
	if (!points_file_stream)
	{
		std::cout << "Failed to open file " << filename << std::endl;
		return false;
	}

	size_t num_points;

	std::string line;
	std::getline(points_file_stream, line);
	std::istringstream line_stream(line);

	line_stream >> num_points;
	points.resize(num_points);

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

		points[point_ID] = new_point;
	}

	std::cout << "Successfully loaded " << points.size() << " points" << std::endl;

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

	uint64_t buff_64;
	features_file_stream.read((char*) &buff_64, sizeof(uint64_t));
	const int num_features = buff_64;

	std::cout << "There are " << num_features << " features to load" << std::endl; 

	uint32_t buff;
	features_file_stream.read((char*) &buff, sizeof(uint32_t));
	num_frames = buff;
	images.resize(num_frames);
	for (int i = 0; i < num_frames; i++)
	{
		images[i].resize(num_cameras);
	}

	features_file_stream.read((char*) &buff, sizeof(uint32_t));
	const int num_observations = buff;

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

		images[frame][sensor].features.push_back(new_feature);

		count++;
	}

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

	int count = 0;
	while(!poses_file_stream.eof() && !poses_file_stream.bad() && count < images.size())
	{
		std::string line;
		std::getline(poses_file_stream, line);
		std::istringstream line_stream(line);

		for (int i = 0; i < num_cameras; i++)
		{
			images[count][i].K = cv::Mat::eye(3, 3, CV_32F);
			images[count][i].R = cv::Mat::eye(3, 3, CV_32F);
			images[count][i].t = cv::Mat::eye(3, 1, CV_32F);

			images[count][i].width = 3840;
			images[count][i].height = 1920;
			images[count][i].K << 2654.375, 0, 1834.875,
							   0, 2654.375, 978.625,
							   0, 0, 1;

			line_stream >> images[count][i].R(0,0) >> images[count][i].R(0,1) >> images[count][i].R(0,2) >> images[count][i].t(0,0)
						>> images[count][i].R(1,0) >> images[count][i].R(1,1) >> images[count][i].R(1,2) >> images[count][i].t(1,0)
						>> images[count][i].R(2,0) >> images[count][i].R(2,1) >> images[count][i].R(2,2) >> images[count][i].t(2,0);

			char buffer[50];
			sprintf(buffer, "%.6d.png", count);
			images[count][i].filename = std::string(buffer);
		}

		count++;
	}	
	
	return true;
}

void InputDataset::FilterPoses(const float min_dist)
{
	int prev = 0;
	filt.push_back(prev); 

	for (int i = 1; i < images.size(); i++)
	{
		const float dist = ComputePoseDistance(images[prev][0].t, images[i][0].t);
		if (dist > min_dist)
		{
			prev = i;
			filt.push_back(prev);
		}
	} 
}

void InputDataset::ComputeDepthRange()
{
	for (const auto& i : filt)
	{
		for (int j = 0; j < 6; j++)
		{
			float max_depth = 0.0f;
			float min_depth = std::numeric_limits<float>::max();

			for (const auto& f : images[i][j].features)
			{
				const int idx = f.point_idx;
				const Point p_cam = TransformPointFromWorldToCam(images[i][j].R, images[i][j].t, points[idx]);
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

			images[i][j].min_depth = min_depth;
			images[i][j].max_depth = max_depth;
		}
	}
}

void InputDataset::BuildFeatureTracks()
{
	for (const auto& i : filt)
	{
		for (int j = 0; j < num_cameras; j++)
		{
			for (const auto& f : images[i][j].features)
			{
				const int uuid = j * num_frames + i;
				points[f.point_idx].image_idx.push_back(uuid);
			}
		}
	}

	const auto lambda_size = [](const Point& p) { return p.image_idx.empty(); };
	points.erase(std::remove_if(points.begin(), points.end(), lambda_size), points.end());
}