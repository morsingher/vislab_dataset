#include "input_dataset.h"

namespace plt = matplotlibcpp;

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

void InputDataset::FilterPoses(const float min_dist)
{
	int prev = 0;
	filt.push_back(prev); 

	for (int i = 1; i < images.size(); i++)
	{
		const float dist = ComputePoseDistance(images[prev].t, images[i].t);
		if (dist > min_dist)
		{
			prev = i;
			filt.push_back(prev);
		}
	} 
}

void InputDataset::BuildFeatureTracks()
{
	for (const auto& i : filt)
	{
		for (const auto& f : images[i].features)
		{
			points[f.point_idx].frame_idx.push_back(i);
		}
	}
}

void InputDataset::ComputeDepthRange()
{
	for (const auto& i : filt)
	{
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

		if (min_depth < 0 || max_depth < 0)
		{
			std::cout << "Weird result on computing depth range" << std::endl;
		}

		images[i].min_depth = min_depth;
		images[i].max_depth = max_depth;
	}
}

void InputDataset::PlotPointCloud()
{
	std::vector<float> x, z;
	for (const auto& p : points)
	{
		x.push_back(p.x);
		z.push_back(p.z);
	}
	plt::plot(x, z, "o ");
	plt::show();
}

void InputDataset::PlotTrajectory()
{
	std::vector<float> x, z;
	for (const auto& img : images)
	{
		x.push_back(img.t(0,0));
		z.push_back(img.t(2,0));
	}
	plt::plot(x, z, "o ");
	plt::show();
}

void InputDataset::PlotFilteredTrajectory()
{
	std::vector<float> x, z;
	for (const auto& i : filt)
	{
		x.push_back(images[i].t(0,0));
		z.push_back(images[i].t(2,0));
	}
	plt::plot(x, z, "o ");
	plt::show();
}