/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "KepointsHarris3dDetector.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/keypoints/harris_3d.h>

namespace Processors {
namespace KepointsHarris3dDetector {

KepointsHarris3dDetector::KepointsHarris3dDetector(const std::string & name) :
		Base::Component(name), radius_search("radius_search", 0.01), radius("radius", 0.5) {
	registerProperty(radius_search);
	registerProperty(radius);

}

KepointsHarris3dDetector::~KepointsHarris3dDetector() {
}

void KepointsHarris3dDetector::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyz.setup(boost::bind(&KepointsHarris3dDetector::computeXYZ, this));
	registerHandler("h_compute_xyz", &h_compute_xyz);
	addDependency("h_compute_xyz", &in_cloud_xyz);

	h_compute_xyzrgb.setup(boost::bind(&KepointsHarris3dDetector::computeXYZRGB, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);

}

bool KepointsHarris3dDetector::onInit() {

	return true;
}

bool KepointsHarris3dDetector::onFinish() {
	return true;
}

bool KepointsHarris3dDetector::onStop() {
	return true;
}

bool KepointsHarris3dDetector::onStart() {
	return true;
}

void KepointsHarris3dDetector::computeXYZ() {
	LOG(LTRACE) << "KepointsHarris3dDetector::computeXYZ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>());

		pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>* detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ,
				pcl::PointXYZI>(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS);

		detector->setNonMaxSupression(true);
		detector->setRadius(radius);
		detector->setRadiusSearch(radius_search);
		detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI>::HARRIS);
		detector->setNumberOfThreads(10);
		detector->setSearchMethod(tree);
		detector->setInputCloud(copy);

		detector->compute(*keypoints_temp);
		std::vector<int> indices = *(detector->getIndices().get());

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());

		pcl::copyPointCloud(*keypoints_temp, *keypoints);

		CLOG(LNOTICE)<< "KepointsHarris3dDetector: input xyz cloud: " << cloud->size() << " points, keypoints : " << keypoints->size();

		out_cloud_xyz.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LTRACE)<< "KepointsHarris3dDetector: empty input xyz cloud";
	}
}

void KepointsHarris3dDetector::computeXYZRGB() {
	LOG(LTRACE) << "KepointsHarris3dDetector::computeXYZRGB";

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>());

		pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>* detector = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,
				pcl::PointXYZI>(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS);

		detector->setNonMaxSupression(true);
		detector->setRadius(radius);
		detector->setRadiusSearch(radius_search);
		detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS);
		detector->setNumberOfThreads(10);
		detector->setSearchMethod(tree);
		detector->setInputCloud(copy);

		detector->compute(*keypoints_temp);
		std::vector<int> indices = *(detector->getIndices().get());

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		pcl::copyPointCloud(*keypoints_temp, *keypoints);

		CLOG(LNOTICE)<< "KepointsHarris3dDetector: input xyzrgb cloud: " << cloud->size() << " points, keypoints : " << keypoints->size();

		out_cloud_xyzrgb.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LTRACE)<< "KepointsHarris3dDetector: empty input xyzrgb cloud";
	}
}

} //: namespace KepointsHarris3dDetector
} //: namespace Processors
