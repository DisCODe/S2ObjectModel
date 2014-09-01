/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "MLS.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/filter.h>

namespace Processors {
namespace MLS {

MLS::MLS(const std::string & name) :
		Base::Component(name), radius_search("radius_search", 0.05)  {
	registerProperty(radius_search);

}

MLS::~MLS() {
}

void MLS::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyzrgb.setup(boost::bind(&MLS::compute, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);
}

bool MLS::onInit() {

	return true;
}

bool MLS::onFinish() {
	return true;
}

bool MLS::onStop() {
	return true;
}

bool MLS::onStart() {
	return true;
}

void MLS::compute() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	if (copy->size() > 0) {

		  // Create a KD-Tree
		  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

		  // Output has the PointNormal type in order to store the normals calculated by MLS
		  pcl::PointCloud<pcl::PointXYZRGBNormal> mls_points;

		  // Init object (second point type is for the normals, even if unused)
		  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;

		  mls.setComputeNormals (true);

		  // Set parameters
		  mls.setInputCloud (copy);
		  mls.setPolynomialFit (true);
		  mls.setSearchMethod (tree);
		  mls.setSearchRadius (radius_search);

		  // Reconstruct
		  mls.process (mls_points);
		  pcl::copyPointCloud (mls_points, *result);


		CLOG(LNOTICE)<< "KepointsIssDetector: input xyzrgb cloud: " << copy->size() << " points, keypoints : " << result->size();

		out_cloud_xyzrgb.write(result);
	} else {
		CLOG(LWARNING)<< "KepointsIssDetector: empty input xyzrgb cloud";
	}
}


} //: namespace MLS
} //: namespace Processors
