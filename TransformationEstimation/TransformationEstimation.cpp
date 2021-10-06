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

	auto srcRow = srcDist.row(0);

	Eigen::VectorXf x = srcRow;
	ArrayXf v = srcRow;
	//std::sort(v.begin(), v.end());
	//std::sort(begin(v), end(v), std::greater<float>());

	//std::sort(begin(vec), end(vec), FooSorter());

	//TODO: Add distance sorting and indices sorting based on it
	cout << "Src distances:" << endl;
	cout << srcDist << endl;
	cout << endl;
	cout << "Tgt distances:" << endl;
	cout << tgtDist << endl;
	cout << endl;

	for (int i = 0; i < src.size(); i++)
	{
		for (int j = 0; j < src.size(); j++)
		{
			if (i != j)
			{
				if (abs(srcDist(i, j) - tgtDist(i, j)) < tolerance)
				{
					ind.push_back(j);
					cout << j << endl;
				}
			}
		}
	}

	//Find correspondence
	if ((srcDist - tgtDist).cwiseAbs().norm() < tolerance)
	{
		cout << "Matrices are equals" << endl;
		cout << endl;
		ind.push_back(0);
		ind.push_back(1);
		ind.push_back(2);
		ind.push_back(3);
		ind.push_back(4);
	}
	else
	{
		cout << "Matrices are not equals" << endl;
		cout << endl;
		ind.push_back(0);
		ind.push_back(2);
		ind.push_back(1);
		ind.push_back(3);
		ind.push_back(4);
	}
}

int main()
{
	PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);
	auto p1 = PointXYZ(0, 0, 0);
	auto p2 = PointXYZ(20.000, 80.042, 2.530);
	auto p3 = PointXYZ(60.000, 10.042, -5.530);
	auto p4 = PointXYZ(-52.0, 63.11, 0.28);
	auto p5 = PointXYZ(-10.00, -13.000, -2.030);
	src->push_back(p1);
	src->push_back(p2);
	src->push_back(p3);
	src->push_back(p4);
	src->push_back(p5);

	PointCloud<PointXYZ>::Ptr tgt(new PointCloud<PointXYZ>);
	auto p5t = PointXYZ(10.000, 20.000, 30.000);
	auto p3t = PointXYZ(-32.456, 90.740, 32.530);
	auto p2t = PointXYZ(45.326, 69.527, 24.470);
	auto p4t = PointXYZ(-71.395, 27.856, 30.280);
	auto p1t = PointXYZ(12.121, 3.737, 27.970);
	tgt->push_back(p1t);
	tgt->push_back(p2t);
	tgt->push_back(p3t);
	tgt->push_back(p4t);
	tgt->push_back(p5t);

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
