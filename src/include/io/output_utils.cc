#include "output_utils.h"

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
