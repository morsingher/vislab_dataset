#include "input_dataset.h"

InputDataset::InputDataset(const Parameters& p) : params(p) {};

bool InputDataset::Load()
{
	std::cout << "Loading points from: " << params.points_file << std::endl;
	if(!LoadPoints())
	{
		std::cout << "Failed to load points!" << std::endl;
		return false;
	}

	std::cout << "Loading features from: " << params.features_file << std::endl;
	if (!LoadFeatures())
	{
		std::cout << "Failed to load features!" << std::endl;
		return false;
	}

	std::cout << "Loading poses from: " << params.poses_file << std::endl;
	if (!LoadPoses())
	{
		std::cout << "Failed to load poses!" << std::endl;
		return false;
	}

	for (int i = 0; i < params.num_cameras; i++)
	{
		if (!ReadCalibrationFile(i))
		{
			std::cout << "Failed to read calibration file for camera " << i << std::endl;
			return false;
		}
	}

	SetFilenames();
	FilterPoses();
	BuildFeatureTracks();
	ComputeDepthRange();

	return true;
}

bool InputDataset::LoadPoints()
{
	std::ifstream points_file_stream(params.points_file, std::ios::in);
	if (!points_file_stream)
	{
		std::cout << "Failed to open file " << params.points_file << std::endl;
		return false;
	}

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

		// new_point.r = 0;
		// new_point.g = 0;
		// new_point.b = 0;
		// new_point.error = 0.0f;

		points[point_ID] = new_point;
	}

	std::cout << "Successfully loaded " << points.size() << " points!" << std::endl;

	return true;
}

bool InputDataset::LoadFeatures()
{
	std::ifstream features_file_stream(params.features_file, std::ios::in);
	if (!features_file_stream)
	{
		std::cout << "Failed to open file " << params.features_file << std::endl;
		return false;
	}

	uint64_t buff_64;
	features_file_stream.read((char*) &buff_64, sizeof(uint64_t));
	const int num_features = buff_64;

	uint32_t buff;
	features_file_stream.read((char*) &buff, sizeof(uint32_t));
	num_frames = buff;
	images.resize(num_frames);
	for (int i = 0; i < num_frames; i++)
	{
		images[i].resize(params.num_cameras);
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

	std::cout << "Successfully loaded " << count << " features!" << std::endl;

	return true;
}

bool InputDataset::LoadPoses()
{
	std::ifstream poses_file_stream(params.poses_file, std::ios::in);
	if (!poses_file_stream)
	{
		std::cout << "Failed to open file " << params.poses_file << std::endl;
		return false;
	}

	// Load poses of camera FC_R

	int count = 0;
	while(!poses_file_stream.eof() && !poses_file_stream.bad() && count < images.size())
	{
		std::string line;
		std::getline(poses_file_stream, line);
		std::istringstream line_stream(line);

		images[count][0].R = cv::Mat::eye(3, 3, CV_32F);
		images[count][0].t = cv::Mat::eye(3, 1, CV_32F);

		line_stream >> images[count][0].R(0,0) >> images[count][0].R(0,1) >> images[count][0].R(0,2) >> images[count][0].t(0,0)
					>> images[count][0].R(1,0) >> images[count][0].R(1,1) >> images[count][0].R(1,2) >> images[count][0].t(1,0)
					>> images[count][0].R(2,0) >> images[count][0].R(2,1) >> images[count][0].R(2,2) >> images[count][0].t(2,0);

		count++;
	}

	// Set other poses (TODO here or below?)

	for (int i = 0; i < count; i++)
	{
		for (int j = 1; j < params.num_cameras; j++)
		{
			images[i][j].R = cv::Mat::eye(3, 3, CV_32F);
			images[i][j].t = cv::Mat::eye(3, 1, CV_32F);
		}
	}

	std::cout << "Successfully loaded " << count << " poses!" << std::endl;	
	
	return true;
}

bool InputDataset::ReadCalibrationFile(const int cam_id)
{
	const std::string filename = params.cameras_folder + cam_dictionary.at(cam_id) + "_R.json";

	std::cout << "Reading calibration data from " << filename << std::endl;

	FILE* fp = fopen(filename.c_str(), "r");
	if (fp == nullptr)
	{
		std::cout << "Couldn't open the file " << filename << std::endl;
		return false;
	}

 	char read_buffer[65536];
	rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer)); 
	rapidjson::Document d;
	d.ParseStream(is);
	fclose(fp);

	const auto& intrinsics = d["params"];
	const double f_x = intrinsics["ku"].GetDouble();
	const double f_y = intrinsics["kv"].GetDouble();
	const double c_x = intrinsics["u0"].GetDouble();
	const double c_y = intrinsics["v0"].GetDouble();
	const int width = intrinsics["w"].GetInt();
	const int height = intrinsics["h"].GetInt();

	cv::Mat_<float> K = cv::Mat::eye(3, 3, CV_32F);
	K << f_x, 0, c_x, 0, f_y, c_y, 0, 0, 1;

	// std::cout << "Image size for camera " << cam_id << ": (" << width << ", " << height << ")" << std::endl;
	// std::cout << "Calibration matrix for camera " << cam_id << ": " << std::endl << K << std::endl;

	for (int i = 0; i < num_frames; i++)
	{
		images[i][cam_id].K = K;
		images[i][cam_id].width = width;
		images[i][cam_id].height = height;
	}

	const auto& extrinsics = d["position"];
	const double x = extrinsics[0].GetDouble();
	const double y = extrinsics[1].GetDouble();
	const double z = extrinsics[2].GetDouble();
	const double roll = extrinsics[3].GetDouble();
	const double pitch = extrinsics[4].GetDouble();
	const double yaw = extrinsics[5].GetDouble();

	if (cam_id == 0)
	{
		t_ref = cv::Mat::zeros(3, 1, CV_32F);
		t_ref << x, y, z;
		R_ref = RotationMatrixFromEulerAngles(roll, pitch, yaw);
	}
	else
	{
		cv::Mat_<float> t_src = cv::Mat::zeros(3, 1, CV_32F);
		t_src << x, y, z;
		cv::Mat_<float> R_src = RotationMatrixFromEulerAngles(roll, pitch, yaw);
		for (int i = 0; i < num_frames; i++)
		{
			images[i][cam_id].t = R_ref.inv() * (t_src - t_ref) + images[i][0].t;
			images[i][cam_id].R = R_ref.inv() * R_src * images[i][0].R;
		}
	}

	return true;
}

void InputDataset::SetFilenames()
{
	for (int i = 0; i < num_frames; i++)
	{
		for (int j = 0; j < params.num_cameras; j++)
		{
			const std::string path = params.images_folder + cam_dictionary.at(j);
			std::stringstream filename;
    		filename << path << "/" << std::setw(8) << std::setfill('0') << i << ".jpg";
    		images[i][j].filename = filename.str();
		}
	}
}

void InputDataset::FilterPoses()
{
	int prev = 0;
	idx_filt.push_back(prev); 

	for (int i = 1; i < images.size(); i++)
	{
		const double dist = ComputePoseDistance(images[prev][0].t, images[i][0].t);
		if (dist > params.min_dist)
		{
			prev = i;
			idx_filt.push_back(prev);
		}
	}
}

void InputDataset::BuildFeatureTracks()
{
	for (const auto& i : idx_filt)
	{
		for (int j = 0; j < params.num_cameras; j++)
		{
			for (int k = 0; k < images[i][j].features.size(); k++)
			{
				const int point_id = images[i][j].features[k].point_idx;
				const int uuid = j * num_frames + i;
				points[point_id].image_idx.push_back(std::make_pair(uuid, k));
			}
		}
	}

	// The lines below are very tricky since they mess up with point indices

	// const auto lambda_size = [](const Point& p) { return p.image_idx.empty(); };
	// points.erase(std::remove_if(points.begin(), points.end(), lambda_size), points.end());
}

void InputDataset::ComputeDepthRange()
{	
	for (const auto& i : idx_filt)
	{
		// for (int j = 0; j < params.num_cameras; j++) // TODO
		for (int j = 0; j < 1; j++)
		{
			std::vector<double> depths;
			for (const auto& f : images[i][j].features)
			{
				const int idx = f.point_idx;
				const Point p_cam = TransformPointFromWorldToCam(images[i][j].R, images[i][j].t, points[idx]);
				depths.push_back(p_cam.z);
			}

			std::sort(depths.begin(), depths.end());
			images[i][j].min_depth = std::min(params.min_depth, 0.75 * depths[0]);
			const int limit_idx = static_cast<int>(0.9f * depths.size());
			images[i][j].max_depth = std::min(params.max_depth, depths[limit_idx]);

			// std::cout << "Depth range for image " << i << ": (" << images[i][j].min_depth << ", " << images[i][j].max_depth << ")" << std::endl;
		}
	}
}