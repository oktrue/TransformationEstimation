// TransformationEstimation.cpp : Defines the entry point for the application.
//

#include "TransformationEstimation.h"

using namespace std;
using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;

void EstimateIndices(const PointCloud<PointXYZ> src, const PointCloud<PointXYZ> tgt, Indices& ind, const float allowance = 0.01)
{
	std::vector<std::vector<float>> srcDistances;
	std::vector<std::vector<float>> tgtDistances;

	//Calc all distances for src point cloud
	for (auto currentPoint : src)
	{
		std::vector<float> distances;

		for (auto point : src)
		{
			auto d = pcl::geometry::distance(currentPoint, point);
			distances.push_back(d);
		}

		srcDistances.push_back(distances);
	}

	//Calc all distances for tgt point cloud
	for (auto currentPoint : tgt)
	{
		std::vector<float> distances;

		for (auto point : tgt)
		{
			auto d = pcl::geometry::distance(currentPoint, point);
			distances.push_back(d);
		}

		tgtDistances.push_back(distances);
	}

	//Find correspondence
	for (auto currentPointDistances : srcDistances)
	{

	}

	Eigen::Matrix<float, 2, 2> a;
	Eigen::Matrix<float, 2, 2> b;
	a(0, 0) = 1.6f;
	a(0, 1) = 1.0f;
	a(1, 0) = 1.0f;
	a(1, 1) = 1.0f;
	b(0, 0) = 1.7f;
	b(0, 1) = 1.0f;
	b(1, 0) = 1.0f;
	b(1, 1) = 1.25f;
	auto c = (a - b).cwiseAbs();

	cout << a << endl;
	cout << b << endl;
	cout << c << endl;

	if (c.norm() < 0.2f)
	{
		cout << "Matrices are equals" << endl;
	}
	else
	{
		cout << "Matrices are not equals" << endl;
	}

	cout << "Finished" << endl;
}

int main()
{
	PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);
	auto p0 = PointXYZ(0, 0, 0);
	auto p1 = PointXYZ(20.000, 80.042, 2.530);
	auto p2 = PointXYZ(60.000, 10.042, -5.530);
	src->push_back(p0);
	src->push_back(p1);
	src->push_back(p2);

	PointCloud<PointXYZ>::Ptr tgt(new PointCloud<PointXYZ>);
	auto p0t = PointXYZ(20, 10, -20);
	auto p2t = PointXYZ(-22.456, 80.740, -17.470);
	auto p1t = PointXYZ(55.326, 59.527, -25.530);
	tgt->push_back(p0t);
	tgt->push_back(p1t);
	tgt->push_back(p2t);

	//indices
	Indices ind;
	ind.push_back(0);
	ind.push_back(2);
	ind.push_back(1);

	EstimateIndices(*src, *tgt, ind);

	//correspondences
	CorrespondenceEstimation<PointXYZ, PointXYZ> ce;
	ce.setInputSource(src);
	ce.setInputTarget(tgt);
	Correspondences all_correspondences;
	ce.determineReciprocalCorrespondences(all_correspondences);

	//svd
	TransformationEstimationSVD<PointXYZ, PointXYZ> te;
	Matrix4f t;
	//te.estimateRigidTransformation(*src, *tgt, all_correspondences, t);
	//te.estimateRigidTransformation(*src, *tgt, t);
	te.estimateRigidTransformation(*src, ind, *tgt, t);
	Matrix3f r = t.topLeftCorner<3, 3>();
	Vector3f e = r.eulerAngles(2, 1, 0);
	e *= 180 / M_PI;
	cout << "Transformation matrix:" << endl;
	cout << t << endl;
	cout << endl;
	cout << "Euler angles:" << endl;
	cout << e << endl;
	cin.get();
	return 0;
}
