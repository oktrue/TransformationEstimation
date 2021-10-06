// TransformationEstimation.cpp : Defines the entry point for the application.
//

#include "TransformationEstimation.h"

using namespace std;
using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;

void estimateIndices(const PointCloud<PointXYZ> src, const PointCloud<PointXYZ> tgt, Indices& ind, const float tolerance = 0.1f)
{
	MatrixXf srcDist(src.size(), src.size());
	MatrixXf tgtDist(src.size(), src.size());

	//Calc all distances for src point cloud
	for (int i = 0; i < src.size(); i++)
	{
		for (int j = 0; j < src.size(); j++)
		{
			srcDist(i, j) = pcl::geometry::distance(src[i], src[j]);
		}
	}

	//Calc all distances for tgt point cloud
	for (int i = 0; i < tgt.size(); i++)
	{
		for (int j = 0; j < tgt.size(); j++)
		{
			tgtDist(i, j) = pcl::geometry::distance(tgt[i], tgt[j]);
		}
	}

	//TODO: Add distance sorting and indices sorting based on it
	cout << srcDist << endl;
	cout << tgtDist << endl;

	//Find correspondence
	if ((srcDist - tgtDist).cwiseAbs().norm() < tolerance)
	{
		cout << "Matrices are equals" << endl;
		ind.push_back(0);
		ind.push_back(1);
		ind.push_back(2);
	}
	else
	{
		cout << "Matrices are not equals" << endl;
		ind.push_back(0);
		ind.push_back(2);
		ind.push_back(1);
	}
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
	estimateIndices(*src, *tgt, ind);

	//svd
	TransformationEstimationSVD<PointXYZ, PointXYZ> te;
	Matrix4f t;
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
