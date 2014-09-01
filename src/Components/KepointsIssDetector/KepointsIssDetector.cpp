/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "KepointsIssDetector.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/filter.h>

namespace Processors {
namespace KepointsIssDetector {

KepointsIssDetector::KepointsIssDetector(const std::string & name) :
		Base::Component(name), radius_search("radius_search", 0.0005), gamma_21("gamma_21", 0.8), gamma_32("gamma_31",
				0.8), model_resolution("model_resolution", 0.001), min_neighbors("min_neighbors", 4), normal_radius(
				"normal_radius", 0.001) {
	registerProperty(radius_search);
	registerProperty(gamma_21);
	registerProperty(gamma_32);
	registerProperty(model_resolution);
	registerProperty(min_neighbors);
	registerProperty(normal_radius);

}

KepointsIssDetector::~KepointsIssDetector() {
}

void KepointsIssDetector::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyz.setup(boost::bind(&KepointsIssDetector::computeXYZ, this));
	registerHandler("h_compute_xyz", &h_compute_xyz);
	addDependency("h_compute_xyz", &in_cloud_xyz);

	h_compute_xyzrgb.setup(boost::bind(&KepointsIssDetector::computeXYZRGB, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);
}

bool KepointsIssDetector::onInit() {

	return true;
}

bool KepointsIssDetector::onFinish() {
	return true;
}

bool KepointsIssDetector::onStop() {
	return true;
}

bool KepointsIssDetector::onStart() {
	return true;
}

void KepointsIssDetector::computeXYZ() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

		pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());

		double iss_salient_radius_ = 6 * model_resolution;
		double iss_non_max_radius_ = 4 * model_resolution;

		iss_detector.setSearchMethod(tree);
		iss_detector.setSalientRadius(iss_salient_radius_);
		iss_detector.setNonMaxRadius(iss_non_max_radius_);
		iss_detector.setThreshold21(gamma_21);
		iss_detector.setThreshold32(gamma_32);
		iss_detector.setNormalRadius(normal_radius);
		//iss_detector.setBorderRadius(iss_border_radius_);
		iss_detector.setMinNeighbors(min_neighbors);
		iss_detector.setNumberOfThreads(5);
		iss_detector.setInputCloud(copy);
		iss_detector.setRadiusSearch(radius_search);
		iss_detector.compute(*keypoints);
		std::vector<int> indices = *(iss_detector.getIndices().get());

		CLOG(LNOTICE)<< "KepointsIssDetector: input xyz cloud: " << cloud->size() << " points, keypoints : " << keypoints->size();

		out_cloud_xyz.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LWARNING)<< "KepointsIssDetector: empty input xyz cloud";
	}
}

void KepointsIssDetector::computeXYZRGB() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

		pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		double iss_salient_radius_ = 6 * model_resolution;
		double iss_non_max_radius_ = 4 * model_resolution;

		iss_detector.setSearchMethod(tree);
		iss_detector.setSalientRadius(iss_salient_radius_);
		iss_detector.setNonMaxRadius(iss_non_max_radius_);
		iss_detector.setThreshold21(gamma_21);
		iss_detector.setThreshold32(gamma_32);
		iss_detector.setNormalRadius(normal_radius);
		//iss_detector.setBorderRadius(iss_border_radius_);
		iss_detector.setMinNeighbors(min_neighbors);
		iss_detector.setNumberOfThreads(5);
		iss_detector.setInputCloud(copy);
		iss_detector.setRadiusSearch(radius_search);
		iss_detector.compute(*keypoints);
		std::vector<int> indices = *(iss_detector.getIndices().get());

		CLOG(LNOTICE)<< "KepointsIssDetector: input xyzrgb cloud: " << cloud->size() << " points, keypoints : " << keypoints->size();

		out_cloud_xyzrgb.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LWARNING)<< "KepointsIssDetector: empty input xyzrgb cloud";
	}
}

} //: namespace KepointsIssDetector
} //: namespace Processors
