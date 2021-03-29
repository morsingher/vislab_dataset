#include "math_utils.h"

float ComputePoseDistance(const cv::Mat_<float>& t1, const cv::Mat_<float>& t2)
{
	return std::sqrt(std::pow(t1(0,0) - t2(0,0), 2) +
					 std::pow(t1(1,0) - t2(1,0), 2) +
					 std::pow(t1(2,0) - t2(2,0), 2));
}

Point TransformPointFromWorldToCam(const cv::Mat_<float>& R, const cv::Mat_<float>& t, const Point& p)
{
	cv::Mat_<float> p_world = cv::Mat::eye(3, 1, CV_32F);
	p_world << p.x, p.y, p.z;
	cv::Mat_<float> p_cam = R.inv() * (p_world - t);
	return Point(p_cam(0,0), p_cam(1,0), p_cam(2,0));
}

cv::Mat_<float> RotationMatrixFromEulerAngles(const double roll, const double pitch, const double yaw)
{
	cv::Mat_<float> R = cv::Mat::eye(3, 3, CV_32F);

	double cr = std::cos(roll);
	double sr = std::sin(roll);
	double cp = std::cos(pitch);
	double sp = std::sin(pitch);
	double cy = std::cos(yaw);
	double sy = std::sin(yaw);

	R << cr * cp, sr * sy - cr * sp * cy, cr * sp * sy + sp * cy,
		 sp, cp * cy, -cp * sy,
		 -sr * cp, sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy; 

	return R;
}

// cv::Point2f ObservePoint(const Point& p, const cv::Mat_<float>& K)
// {
// 	const float u = K(0,0) * (p.x / p.z) + K(0,2);
// 	const float v = K(1,1) * (p.y / p.z) + K(1,2);
// 	return cv::Point2f(u, v);
// }

// Point ProjectPointAtDepth(const cv::Point2f& pixel, const cv::Mat_<float>& K, const float depth)
// {
// 	Point p;
// 	p.x = depth * (pixel.x - K(0,2)) / K(0,0);
// 	p.y = depth * (pixel.y - K(1,2)) / K(1,1);
// 	p.z = depth;
// 	return p;
// }

// Point TransformPointFromCamToWorld(const cv::Mat_<float>& R, const cv::Mat_<float>& t, const Point& p)
// {
// 	cv::Mat_<float> p_cam = cv::Mat::eye(3, 1, CV_32F);
// 	p_cam << p.x, p.y, p.z;
// 	cv::Mat_<float> p_world = R * p_cam + t;
// 	return Point(p_world(0,0), p_world(1,0), p_world(2,0));
// }

// float ComputeTriangulationAngle(const Point& p, const cv::Mat_<float>& t_ref, const cv::Mat_<float>& t_src)
// {
// 	cv::Vec3f v1(t_ref(0,0) - p.x, t_ref(1,0) - p.y, t_ref(2,0) - p.z);
// 	cv::Vec3f v2(t_src(0,0) - p.x, t_src(1,0) - p.y, t_src(2,0) - p.z);
// 	cv::Vec3f v1_norm = cv::normalize(v1);
// 	cv::Vec3f v2_norm = cv::normalize(v2);
// 	return (180.0f / M_PI) * acosf(v1_norm.dot(v2_norm));
// }

// Point TransformPointFromFLUToRDF(const Point& p)
// {
// 	return Point(-p.y, -p.z, p.x);
// }

// Quaternion QuaternionFromRotationMatrix(const cv::Mat_<float>& R)
// {
// 	Quaternion q;
// 	float trace = R(0,0) + R(1,1) + R(2,2);
	
// 	if (trace > 0.0)
// 	{
// 		float s = std::sqrt(trace + 1.0);
// 		q[3] = s * 0.5;
// 		s = 0.5 / s;
// 		q[0] = s * (R(2,1) - R(1,2));
// 		q[1] = s * (R(0,2) - R(2,0));
// 		q[2] = s * (R(1,0) - R(0,1));
// 	}
// 	else
// 	{
// 		int i = R(0,0) < R(1,1) ? (R(1,1) < R(2,2) ? 2 : 1) : (R(0,0) < R(2,2) ? 2 : 0);
// 		int j = (i + 1) % 3;
// 		int k = (i + 2) % 3;
// 		float s = std::sqrt(R(i,i) - R(j,j) - R(k,k) + 1.0);
// 		q[i] = s * 0.5;
// 		s = 0.5 / s;

// 		q[3] = s * (R(k,j) - R(j,k));
// 		q[j] = s * (R(j,i) - R(i,j));
// 		q[k] = s * (R(k,i) - R(i,k));
// 	}

// 	return q;
// }
