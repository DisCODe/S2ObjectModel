/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "KepointsSusanDetector.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/keypoints/susan.h>
#include <pcl/filters/filter.h>

namespace Processors {
namespace KepointsSusanDetector {

KepointsSusanDetector::KepointsSusanDetector(const std::string & name) :
		Base::Component(name), radius_search("radius_search", 0.01), radius("radius", 0.5) {
	registerProperty(radius_search);
	registerProperty(radius);

}

KepointsSusanDetector::~KepointsSusanDetector() {
}

void KepointsSusanDetector::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyzrgb.setup(boost::bind(&KepointsSusanDetector::computeXYZRGB, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);

}

bool KepointsSusanDetector::onInit() {

	return true;
}

bool KepointsSusanDetector::onFinish() {
	return true;
}

bool KepointsSusanDetector::onStop() {
	return true;
}

bool KepointsSusanDetector::onStart() {
	return true;
}

void KepointsSusanDetector::computeXYZRGB() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());

	LOG(LWARNING) << "KepointsSusanDetector::computeXYZRGB";

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(new pcl::PointCloud<pcl::PointXYZI>());

		pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* detector = new pcl::SUSANKeypoint<pcl::PointXYZRGB,
				pcl::PointXYZRGB>();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
		detector->setInputCloud(copy);
		detector->setSearchMethod(tree);
		detector->setNonMaxSupression(true);
		detector->setRadius(radius);
		detector->setRadiusSearch(radius_search);
		detector->compute(*keypoints);
		std::vector<int> indices = *(detector->getIndices().get());

		CLOG(LNOTICE)<< "KepointsSusanDetector: input xyzrgb cloud: " << cloud->size() << " points, keypoints : " << keypoints->size();

		out_cloud_xyzrgb.write(keypoints);
		out_indices.write(indices);
	} else {
		CLOG(LWARNING)<< "KepointsHarris3dDetector: empty input xyzrgb cloud";
	}
}

} //: namespace KepointsSusanDetector
} //: namespace Processors
