// TransformationEstimation.cpp : Defines the entry point for the application.
//

#include "TransformationEstimation.h"

using namespace std;
using namespace pcl;
using namespace pcl::registration;
using namespace Eigen;

int main()
{
	PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);
	auto p1 = PointXYZ(0, 0, 0);
	auto p2 = PointXYZ(20.000, 80.042, 2.530);
	auto p3 = PointXYZ(60.000, 10.042, -5.530);
	src->push_back(p1);
	src->push_back(p2);
	src->push_back(p3);

	PointCloud<PointXYZ>::Ptr tgt(new PointCloud<PointXYZ>);
	auto p1t = PointXYZ(20, 10, -20);
	auto p2t = PointXYZ(-22.456, 80.740, -17.470);
	auto p3t = PointXYZ(55.326, 59.527, -25.530);
	tgt->push_back(p1t);
	tgt->push_back(p2t);
	tgt->push_back(p3t);

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
	te.estimateRigidTransformation(*src, *tgt, t);
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
