// TransformationEstimation.cpp : Defines the entry point for the application.
//

#include "TransformationEstimation.h"

using namespace std;
using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;

void EstimateIndices(const PointCloud<PointXYZ> src, const PointCloud<PointXYZ> tgt, Indices& ind, const float tolerance)
{
	MatrixXf srcDist(src.size(), src.size());
	MatrixXf tgtDist(src.size(), src.size());

	//calc all distances for src point cloud
	for (int i = 0; i < src.size(); i++)
	{
		for (int j = 0; j < src.size(); j++)
		{
			srcDist(i, j) = pcl::geometry::distance(src[i], src[j]);
		}
	}

	//calc all distances for tgt point cloud
	for (int i = 0; i < tgt.size(); i++)
	{
		for (int j = 0; j < tgt.size(); j++)
		{
			tgtDist(i, j) = pcl::geometry::distance(tgt[i], tgt[j]);
		}
	}

	cout << "src distances:" << endl;
	cout << srcDist << endl;
	cout << endl;
	cout << "tgt distances:" << endl;
	cout << tgtDist << endl;
	cout << endl;
	cout << "correspondence:" << endl;

	//find correspondence
	for (int i = 0; i < tgt.size(); i++)
	{
		Eigen::VectorXf vTgt = tgtDist.row(i);
		std::vector<float> vecTgt(&vTgt[0], vTgt.data() + vTgt.cols() * vTgt.rows());
		std::sort(begin(vecTgt), end(vecTgt), std::greater<float>());
		Eigen::VectorXf vTgtSorted = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(vecTgt.data(), vecTgt.size());

		for (int j = 0; j < src.size(); j++)
		{
			Eigen::VectorXf vSrc = srcDist.row(j);
			std::vector<float> vecSrc(&vSrc[0], vSrc.data() + vSrc.cols() * vSrc.rows());
			std::sort(begin(vecSrc), end(vecSrc), std::greater<float>());
			Eigen::VectorXf vSrcSorted = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(vecSrc.data(), vecSrc.size());

			if ((vTgtSorted - vSrcSorted).cwiseAbs().norm() < tolerance)
			{
				ind.push_back(j);
				cout << j << endl;
			}
		}
	}
	cout << endl;
}

extern "C" __declspec(dllexport) void EstimateTransformation(float points[][3], int count, float tolerance = 15.0)
{
	PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);

	for (int i = 0; i < count; i++)
	{
		src->push_back(PointXYZ(points[i][0], points[i][1], points[i][2]));
	}

	PointCloud<PointXYZ>::Ptr tgt(new PointCloud<PointXYZ>);
	auto p0t = PointXYZ(4773.6846, -1567.714, 836.689);
	auto p1t = PointXYZ(1064.5259, 1966.0875, -187.70764);
	auto p2t = PointXYZ(5249.238, 1048.2821, -234.27676);
	//auto p3t = PointXYZ(-71.395, 27.856, 30.280);
	//auto p4t = PointXYZ(12.121, 3.737, 27.970);
	tgt->push_back(p0t);
	tgt->push_back(p1t);
	tgt->push_back(p2t);
	//tgt->push_back(p3t);
	//tgt->push_back(p4t);

	//indices
	Indices ind;
	EstimateIndices(*src, *tgt, ind, tolerance);

	//svd
	TransformationEstimationSVD<PointXYZ, PointXYZ> te;
	Matrix4f t;
	te.estimateRigidTransformation(*src, ind, *tgt, t);
	Matrix3f r = t.topLeftCorner<3, 3>();
	Vector3f e = r.eulerAngles(2, 1, 0);
	e *= 180 / M_PI;
	cout << "transformation matrix:" << endl;
	cout << t << endl;
	cout << endl;
	cout << "euler angles:" << endl;
	cout << e << endl;
}

int main()
{
	float src[][3] = 
	{
		{ -52.0, 63.11, 0.28 },
		{ 60.000, 10.042, -5.530 },
		{ 0, 0, 0 },
		{ 20.000, 80.042, 2.530 },
		{ -10.00, -13.000, -2.030 }
	};
	 
	EstimateTransformation(src, 5);

	cin.get();
	return 0;
}
