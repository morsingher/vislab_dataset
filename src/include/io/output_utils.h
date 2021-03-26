#ifndef MVC_OUTPUT_UTILS_H
#define MVC_OUTPUT_UTILS_H

bool WriteColmapFormat(const std::vector<Cluster>& clusters, const Parameters& params);
bool WriteACMPFormat(const std::vector<Cluster>& clusters, const Parameters& params);
bool WritePatchMatchNetFormat(const std::vector<Cluster>& clusters, const Parameters& params);

// 	bool WriteClustersFiles(const std::string& output_path, const int num_neighbors);
	
// 	bool WriteCamerasFiles(const std::string& path, const int idx);
// 	bool WriteNeighborsFile(const std::string& path, const int idx, const int num_neighbors);
// 	bool WriteImages(const std::string& path, const int idx);

// 	bool WriteColmapFiles(const std::string& output_path);
// 	bool WriteColmapCamerasFile(const std::string& path, const int idx);
// 	bool WriteColmapImagesFile(const std::string& path, const int idx);
// 	bool WriteColmapPointsFile(const std::string& path, const int idx);

#endif