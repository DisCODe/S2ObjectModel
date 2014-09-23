/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SingleS20MMatcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Types/Features.hpp"
#include <Types/MergeUtils.hpp>


namespace Processors {
namespace SingleS20MMatcher {

class SHOTonlyXYZRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSHOT> {
	/// Templatiated number of SIFT descriptor dimensions.
	using pcl::PointRepresentation<PointXYZSHOT>::nr_dimensions_;

public:
	SHOTonlyXYZRepresentation() {
		// Define the number of dimensions.
		nr_dimensions_ = 3;
		trivial_ = false;
	}

	/// Overrides the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointXYZSHOT &p, float * out) const {
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
	}
};

class SHOTonlyDescriptorRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSHOT> {
	/// Templatiated number of SIFT descriptor dimensions.
	using pcl::PointRepresentation<PointXYZSHOT>::nr_dimensions_;

public:
	SHOTonlyDescriptorRepresentation() {
		// Define the number of dimensions.
		nr_dimensions_ = 352;
		trivial_ = false;
	}

	/// Overrides the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointXYZSHOT &p, float * out) const {
		for (int i = 0; i < 352; ++i) {
			out[i] = p.descriptor[i];
		}
	}
};

void computeCorrespondences(const pcl::PointCloud<PointXYZSHOT>::ConstPtr &cloud_src,
		const pcl::PointCloud<PointXYZSHOT>::ConstPtr &cloud_trg, const pcl::CorrespondencesPtr& correspondences) {
	pcl::registration::CorrespondenceEstimation<PointXYZSHOT, PointXYZSHOT> correst;
	SHOTonlyDescriptorRepresentation::Ptr point_representation(new SHOTonlyDescriptorRepresentation());
	correst.setPointRepresentation(point_representation);
	correst.setInputSource(cloud_src);
	correst.setInputTarget(cloud_trg);
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences);

}

SingleS20MMatcher::SingleS20MMatcher(const std::string & name) :
		Base::Component(name) {

}

SingleS20MMatcher::~SingleS20MMatcher() {
}

void SingleS20MMatcher::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_model_xyzrgb", &in_model_xyzrgb);
	registerStream("in_model_xyzshot", &in_model_xyzshot);
	registerStream("in_model_xyzsift", &in_model_xyzsift);

	registerStream("in_xyzrgb", &in_xyzrgb);
	registerStream("in_xyzshot", &in_xyzshot);
	registerStream("in_xyzsift", &in_xyzsift);

	registerStream("out_source_keypoints_xyzshot", &out_source_keypoints_xyzshot);
	registerStream("out_target_keypoints_xyzshot", &out_target_keypoints_xyzshot);

	registerStream("out_target_keypoints_xyzsift", &out_target_keypoints_xyzsift);
	registerStream("out_source_keypoints_xyzsift", &out_source_keypoints_xyzsift);

	registerStream("out_source", &out_source);
	registerStream("out_target", &out_target);
	registerStream("out_correspondences_sift", &out_correspondences_sift);
	registerStream("out_correspondences_shot", &out_correspondences_shot);

	// Register handlers
	h_matchAll.setup(boost::bind(&SingleS20MMatcher::matchAll, this));
	registerHandler("h_matchAll", &h_matchAll);
	addDependency("h_matchAll", &in_xyzrgb);
	addDependency("h_matchAll", &in_xyzsift);
	addDependency("h_matchAll", &in_xyzshot);

	h_matchSifts.setup(boost::bind(&SingleS20MMatcher::matchSifts, this));
	registerHandler("h_matchSifts", &h_matchSifts);
	addDependency("h_matchSifts", &in_xyzrgb);
	addDependency("h_matchSifts", &in_xyzsift);

	h_matchShots.setup(boost::bind(&SingleS20MMatcher::matchShots, this));
	registerHandler("h_matchShots", &h_matchShots);
	addDependency("h_matchShots", &in_xyzrgb);
	addDependency("h_matchShots", &in_xyzshot);

	h_refreshModel.setup(boost::bind(&SingleS20MMatcher::refreshModel, this));
	registerHandler("h_refreshModel", &h_refreshModel);
	addDependency("h_refreshModel", &in_model_xyzrgb);
	addDependency("h_refreshModel", &in_model_xyzshot);
	addDependency("h_refreshModel", &in_model_xyzsift);
}

bool SingleS20MMatcher::onInit() {

	return true;
}

bool SingleS20MMatcher::onFinish() {
	return true;
}

bool SingleS20MMatcher::onStop() {
	return true;
}

bool SingleS20MMatcher::onStart() {
	return true;
}

void SingleS20MMatcher::refreshModel() {
	rgb = in_model_xyzrgb.read();
	shots = in_model_xyzshot.read();
	sifts = in_model_xyzsift.read();
}

void SingleS20MMatcher::matchAll() {

	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_xyzsift.read();
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = in_xyzshot.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_xyzrgb.read();

	pcl::CorrespondencesPtr correspondences_shot(new pcl::Correspondences());
	computeCorrespondences(cloud_xyzshot, shots, correspondences_shot);

	pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences());
	MergeUtils::computeCorrespondences(cloud_xyzsift, sifts, correspondences_sift);

	out_source_keypoints_xyzshot.write(cloud_xyzshot);
	out_source_keypoints_xyzsift.write(cloud_xyzsift);
	out_source.write(cloud_xyzrgb);

	out_target.write(rgb);
	out_target_keypoints_xyzshot.write(shots);
	out_target_keypoints_xyzsift.write(sifts);

	out_correspondences_sift.write(correspondences_sift);
	out_correspondences_shot.write(correspondences_shot);
}

void SingleS20MMatcher::matchSifts() {
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_xyzsift.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_xyzrgb.read();

	pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences());
	MergeUtils::computeCorrespondences(cloud_xyzsift, sifts, correspondences_sift);

	out_source_keypoints_xyzsift.write(cloud_xyzsift);
	out_source.write(cloud_xyzrgb);

	out_target.write(rgb);
	out_target_keypoints_xyzsift.write(sifts);

	out_correspondences_sift.write(correspondences_sift);
}

void SingleS20MMatcher::matchShots() {
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = in_xyzshot.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_xyzrgb.read();

	pcl::CorrespondencesPtr correspondences_shot(new pcl::Correspondences());
	computeCorrespondences(cloud_xyzshot, shots, correspondences_shot);

	out_source_keypoints_xyzshot.write(cloud_xyzshot);
	out_source.write(cloud_xyzrgb);

	out_target.write(rgb);
	out_target_keypoints_xyzshot.write(shots);

	out_correspondences_shot.write(correspondences_shot);
}



} //: namespace SingleS20MMatcher
} //: namespace Processors
