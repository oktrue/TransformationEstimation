// TransformationEstimation.cpp : Defines the entry point for the application.
//

#include "TransformationEstimation.h"

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ> src, tgt;
	auto p1 = pcl::PointXYZ(0, 0, 0);
	auto p2 = pcl::PointXYZ(20.000, 80.042, 2.530);
	auto p3 = pcl::PointXYZ(60.000, 10.042, -5.530);
	src.push_back(p1);
	src.push_back(p2);
	src.push_back(p3);

	auto p3t = pcl::PointXYZ(20, 10, -20);
	auto p1t = pcl::PointXYZ(-22.456, 80.740, -17.470);
	auto p2t = pcl::PointXYZ(55.326, 59.527, -25.530);
	tgt.push_back(p1t);
	tgt.push_back(p2t);
	tgt.push_back(p3t);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(src));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>(tgt));

	//correspondence
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
	est.setInputSource(cloud_in);
	est.setInputTarget(cloud_out);
	pcl::Correspondences all_correspondences;
	est.determineReciprocalCorrespondences(all_correspondences);

	//svd
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 R;
	te.estimateRigidTransformation(src, tgt, all_correspondences, R);
	Eigen::Matrix3f b = R.topLeftCorner<3, 3>();
	auto euler = b.eulerAngles(2, 1, 0);
	euler *= 180 / M_PI;
	std::cout << R << std::endl;
	std::cout << euler << std::endl;

	return 0;
}
