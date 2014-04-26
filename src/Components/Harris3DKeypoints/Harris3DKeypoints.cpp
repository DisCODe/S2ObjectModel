/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "Harris3DKeypoints.hpp"
#include "Common/Logger.hpp"

#include <pcl/keypoints/harris_3d.h>
#include <boost/bind.hpp>

namespace Processors {
namespace Harris3DKeypoints {

Harris3DKeypoints::Harris3DKeypoints(const std::string & name) :
		Base::Component(name) {

}

Harris3DKeypoints::~Harris3DKeypoints() {
}

void Harris3DKeypoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud", &in_cloud);
	registerStream("out_keypoints", &out_keypoints);

	h_compute.setup(boost::bind(&Harris3DKeypoints::compute, this));
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud);

}

bool Harris3DKeypoints::onInit() {

	return true;
}

bool Harris3DKeypoints::onFinish() {
	return true;
}

bool Harris3DKeypoints::onStop() {
	return true;
}

bool Harris3DKeypoints::onStart() {
	return true;
}

void Harris3DKeypoints::compute() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*copy, *cloud);

	// ERROR:
	//INFO: Loading components from S2ObjectModel
	//INFO: ComponentFactory: Library open error: libHarris3DKeypoints.so: undefined symbol: _ZN3pcl7PCLBaseINS_8PointXYZEE13setInputCloudERKN5boost10shared_ptrIKNS_10PointCloudIS1_EEEE


	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>* detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI>::HARRIS);
	detector->setNonMaxSupression(true);
	detector->setRadius(20);
	detector->setInputCloud(copy);

	detector->compute(*keypoints_temp);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*keypoints, *keypoints_temp);

	CLOG(LNOTICE) << " keypoints_temp size :" << keypoints_temp->size() << "\n";
	CLOG(LNOTICE) << " keypoints size :" << keypoints->size() << "\n";


	out_keypoints.write(keypoints);
}

} //: namespace Harris3DKeypoints
} //: namespace Processors
