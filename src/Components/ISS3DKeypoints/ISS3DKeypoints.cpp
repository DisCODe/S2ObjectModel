/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "ISS3DKeypoints.hpp"
#include "Common/Logger.hpp"

#include <pcl/keypoints/iss_3d.h>

namespace Processors {
namespace ISS3DKeypoints {

ISS3DKeypoints::ISS3DKeypoints(const std::string & name) :
		Base::Component(name)  {

}

ISS3DKeypoints::~ISS3DKeypoints() {
}

void ISS3DKeypoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud", &in_cloud);
	registerStream("out_keypoints", &out_keypoints);

	h_compute.setup(boost::bind(&ISS3DKeypoints::compute, this));
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud);

}

bool ISS3DKeypoints::onInit() {

	return true;
}

bool ISS3DKeypoints::onFinish() {
	return true;
}

bool ISS3DKeypoints::onStop() {
	return true;
}

bool ISS3DKeypoints::onStart() {
	return true;
}

void ISS3DKeypoints::compute() {

	// TODO properties

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud, *copy);

	LOG(LNOTICE)<< "ISS3DKeypoints: copy size :" << copy->size();
	LOG(LNOTICE)<< "ISS3DKeypoints: cloud size :" << cloud->size();

	//
	//  ISS3D parameters
	//
	double iss_salient_radius_;
	double iss_non_max_radius_;
	double iss_gamma_21_ (0.975);
	double iss_gamma_32_ (0.975);
	double iss_min_neighbors_ (20);
	int iss_threads_ (10);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	// Fill in the model cloud

	double model_resolution = 0.01;

	// Compute model_resolution

	iss_salient_radius_ = 6 * model_resolution;
	iss_non_max_radius_ = 4 * model_resolution;

	//
	// Compute keypoints
	//
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

	iss_detector.setSearchMethod (tree);
	iss_detector.setSalientRadius (iss_salient_radius_);
	iss_detector.setNonMaxRadius (iss_non_max_radius_);
	iss_detector.setThreshold21 (iss_gamma_21_);
	iss_detector.setThreshold32 (iss_gamma_32_);
	iss_detector.setMinNeighbors (iss_min_neighbors_);
	iss_detector.setNumberOfThreads (iss_threads_);
	iss_detector.setInputCloud (cloud);
	iss_detector.compute (*keypoints);

	CLOG(LERROR) << "ISS3DKeypoints: keypoints: " << keypoints->size();
	out_keypoints.write(keypoints);
}

} //: namespace ISS3DKeypoints
} //: namespace Processors
