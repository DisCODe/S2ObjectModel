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

typedef pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> HarrisKeypoint;

namespace Processors {
namespace Harris3DKeypoints {

Harris3DKeypoints::Harris3DKeypoints(const std::string & name) :
				Base::Component(name),
				radius_search("radius_search", 0.01),
				radius("radius", 0.5) {
				registerProperty(radius_search);
				registerProperty(radius);
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(
			new pcl::PointCloud<pcl::PointXYZ>());

	// Remove NaNs.
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indices);

	LOG(LNOTICE)<< "Harris3DKeypoints: copy size :" << copy->size();
	LOG(LNOTICE)<< "Harris3DKeypoints: cloud size :" << cloud->size();

	if (copy->size() > 0) {

		CLOG(LNOTICE)<< "Harris3DKeypoints: : computing keypoints... ";

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(
				new pcl::PointCloud<pcl::PointXYZI>());

		HarrisKeypoint* detector = new HarrisKeypoint(HarrisKeypoint::HARRIS);

		detector->setNonMaxSupression(true);
		detector->setRadius(radius);
		detector->setRadiusSearch(radius_search);
		detector->setMethod(HarrisKeypoint::HARRIS);
	//	detector->setKSearch();
		detector->setNumberOfThreads(10);
		detector->setSearchMethod(tree);
	//	detector->setThreshold();
	//	detector->use_indices_ = false;
		detector->setInputCloud(copy);


		detector->compute(*keypoints_temp);

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(
				new pcl::PointCloud<pcl::PointXYZ>());

		pcl::copyPointCloud(*keypoints_temp, *keypoints);

		CLOG(LNOTICE)<< " keypoints size :" << keypoints->size();
		out_keypoints.write(keypoints);
	}
}

} //: namespace Harris3DKeypoints
} //: namespace Processors
