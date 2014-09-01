/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "UniformSampling.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/filters/filter.h>
#include <pcl/keypoints/uniform_sampling.h>

namespace Processors {
namespace UniformSampling {

UniformSampling::UniformSampling(const std::string & name) :
		Base::Component(name), radius_search("radius_search", 0.01) {
	registerProperty(radius_search);

}

UniformSampling::~UniformSampling() {
}

void UniformSampling::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyz", &out_cloud_xyz);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyz.setup(boost::bind(&UniformSampling::computeXYZ, this));
	registerHandler("h_compute_xyz", &h_compute_xyz);
	addDependency("h_compute_xyz", &in_cloud_xyz);

	h_compute_xyzrgb.setup(boost::bind(&UniformSampling::computeXYZRGB, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);

}

bool UniformSampling::onInit() {

	return true;
}

bool UniformSampling::onFinish() {
	return true;
}

bool UniformSampling::onStop() {
	return true;
}

bool UniformSampling::onStart() {
	return true;
}

void UniformSampling::computeXYZ() {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>());

		pcl::PointCloud<int> sampled_indices;
		pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
		uniform_sampling.setInputCloud(copy);
		uniform_sampling.setRadiusSearch(radius_search);
		uniform_sampling.compute(sampled_indices);
		pcl::copyPointCloud(*copy, sampled_indices.points, *keypoints);

		std::vector<int> indices = *(uniform_sampling.getIndices().get());

		out_cloud_xyz.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LWARNING)<< "KepointsHarris3dDetector: empty input xyz cloud";
	}
}

void UniformSampling::computeXYZRGB() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());

		pcl::PointCloud<int> sampled_indices;
		pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
		uniform_sampling.setInputCloud(copy);
		uniform_sampling.setRadiusSearch(radius_search);
		uniform_sampling.compute(sampled_indices);
		pcl::copyPointCloud(*copy, sampled_indices.points, *keypoints);

		std::vector<int> indices = *(uniform_sampling.getIndices().get());

		out_cloud_xyzrgb.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LWARNING)<< "KepointsHarris3dDetector: empty input xyz cloud";
	}
}

} //: namespace UniformSapmling
} //: namespace Processors
