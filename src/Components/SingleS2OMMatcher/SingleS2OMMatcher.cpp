/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SingleS2OMMatcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include "Common/Logger.hpp"
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
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/filters/filter.h>

//
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

namespace Processors {
namespace SingleS2OMMatcher {

class SIFTOnlyFeatureRepresentation: public pcl::DefaultFeatureRepresentation<PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
	using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;
public:
	SIFTOnlyFeatureRepresentation() {
		// Define the number of dimensions
		nr_dimensions_ = 128;
		trivial_ = false;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointXYZSIFT &p, float * out) const {
		//This representation is only for determining correspondences (not for use in Kd-tree for example - so use only SIFT part of the point
		for (register int i = 0; i < 128; i++)
			out[i] = p.descriptor[i]; //p.descriptor.at<float>(0, i) ;
		//std::cout << "SIFTFeatureRepresentation:copyToFloatArray()" << std::endl ;
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

SingleS2OMMatcher::SingleS2OMMatcher(const std::string & name) :
		Base::Component(name),  RANSAC_MaximumIterations("ransac.max_iterations", 100),
		RANSAC_InlierThreshold("ransac.inlier_threshold",0.5),
		SHOT_maxDistance("shot.max_disance", 2) {

	registerProperty(RANSAC_MaximumIterations);
	registerProperty(RANSAC_InlierThreshold);
	registerProperty(SHOT_maxDistance);
}

SingleS2OMMatcher::~SingleS2OMMatcher() {
}

void SingleS2OMMatcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_models", &in_models);

	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);

	registerStream("out_correspondeces_sift", &out_correspondeces_sift);
	registerStream("out_correspondeces_sift_trans", &out_correspondeces_sift_trans);
	registerStream("out_correspondeces_sift_source_keypoints", &out_correspondeces_sift_source_keypoints);
	registerStream("out_correspondeces_sift_target_keypoints", &out_correspondeces_sift_target_keypoints);

	registerStream("out_correspondeces_shot", &out_correspondeces_shot);
	registerStream("out_correspondeces_shot_trans", &out_correspondeces_shot_trans);
	registerStream("out_correspondeces_shot_source_keypoints", &out_correspondeces_shot_source_keypoints);
	registerStream("out_correspondeces_shot_target_keypoints", &out_correspondeces_shot_target_keypoints);

	registerStream("out_correspondeces_source_cloud", &out_correspondeces_source_cloud);
	registerStream("out_correspondeces_target_cloud", &out_correspondeces_target_cloud);

	registerStream("out_correspondeces_common", &out_correspondeces_common);
	registerStream("out_correspondeces_common_source_keypoints", &out_correspondeces_common_source_keypoints);
	registerStream("out_correspondeces_common_target_keypoints", &out_correspondeces_common_target_keypoints);

	// Register handlers
	h_readModels.setup(boost::bind(&SingleS2OMMatcher::readModels, this));
	registerHandler("readModels", &h_readModels);
	addDependency("readModels", &in_models);
	h_match.setup(boost::bind(&SingleS2OMMatcher::match, this));
	registerHandler("match", &h_match);
	addDependency("match", &in_cloud_xyzsift);
	addDependency("match", &in_cloud_xyzrgb);
	addDependency("match", &in_cloud_xyzshot);
}

bool SingleS2OMMatcher::onInit() {

	return true;
}

bool SingleS2OMMatcher::onFinish() {
	return true;
}

bool SingleS2OMMatcher::onStop() {
	return true;
}

bool SingleS2OMMatcher::onStart() {
	return true;
}

void SingleS2OMMatcher::readModels() {
	CLOG(LWARNING)<< "SingleS2OMMatcher::readModels";
	for( int i = 0; i<models.size(); i++) {
		delete models[i];
	}
	models.clear();
	std::vector<AbstractObject*> abstractObjects = in_models.read();
	for( int i = 0; i<abstractObjects.size(); i++) {
		CLOG(LTRACE)<<"Read model: " << abstractObjects[i]->name;
		S2ObjectModel *model = dynamic_cast<S2ObjectModel*>(abstractObjects[i]);
		if(model!=NULL)
		models.push_back(model);
		else
		CLOG(LWARNING) << "Cannot read model"<< abstractObjects[i]->name;
	}
	CLOG(LWARNING)<< "SingleS2OMMatcher::models.size " << models.size();

}

void SingleS2OMMatcher::match() {
	CLOG(LWARNING)<< "SingleS2OMMatcher::match, models :" << models.size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = in_cloud_xyzshot.read();
	for (int i = 0; i < models.size(); ++i) {
		matchModel(*models[i], cloud_xyzrgb, cloud_xyzsift, cloud_xyzshot);
	}
	return;
}

void SingleS2OMMatcher::matchModel(S2ObjectModel model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb,
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift, pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot) {
	CLOG(LWARNING)<< "SingleS2OMMatcher::matchModel " << model.name;

	Types::HomogMatrix shotTransform;
	Types::HomogMatrix siftTransform;
	pcl::Correspondences temp;

	pcl::CorrespondencesPtr bestSiftCorrs(new pcl::Correspondences());

	// compute and reject shot correspondences
	//pcl::CorrespondencesPtr shotCorrs = computeSHOTCorrespondences(cloud_xyzshot, model.cloud_xyzshot);
	pcl::CorrespondencesPtr shotCorrs = computeSHOTCorrespondences(model.cloud_xyzshot, cloud_xyzshot);
	CLOG(LWARNING)<< "SingleS2OMMatcher::shotCorrs " << shotCorrs->size();

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSHOT> shotRANSAC;
	shotRANSAC.setInputSource(model.cloud_xyzshot);
	shotRANSAC.setInputTarget(cloud_xyzshot);
	shotRANSAC.setInlierThreshold(RANSAC_InlierThreshold);
	shotRANSAC.setMaximumIterations(RANSAC_MaximumIterations);
	shotRANSAC.setInputCorrespondences(shotCorrs);
	shotRANSAC.getCorrespondences(temp);
	shotTransform.setElements(shotRANSAC.getBestTransformation());

	pcl::CorrespondencesPtr bestShotCorrs(new pcl::Correspondences(temp));
	CLOG(LWARNING)<< "SingleS2OMMatcher::bestShotCorrs " << bestShotCorrs->size();

	// compute and reject sift correspondences
	pcl::CorrespondencesPtr siftCorrs = computeSIFTCorrespondences(model.cloud_xyzsift, cloud_xyzsift);

	CLOG(LWARNING)<< "SingleS2OMMatcher::siftCorrs " << siftCorrs->size();
	pcl::registration::CorrespondenceRejectorSampleConsensus<PointXYZSIFT> siftRANSAC;
	siftRANSAC.setInputSource(model.cloud_xyzsift);
	siftRANSAC.setInputTarget(cloud_xyzsift);
	siftRANSAC.setInlierThreshold(RANSAC_InlierThreshold);
	siftRANSAC.setMaximumIterations(RANSAC_MaximumIterations);
	siftRANSAC.setInputCorrespondences(siftCorrs);
	siftRANSAC.getCorrespondences(*bestSiftCorrs);
	siftTransform.setElements(siftRANSAC.getBestTransformation());


	bestSiftCorrs = siftCorrs;
	CLOG(LWARNING)<< "SingleS2OMMatcher::bestSiftCorrs " << bestSiftCorrs->size();

	// prepare xyz clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr shot_source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*model.cloud_xyzshot, *shot_source_keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr shot_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud_xyzshot, *shot_target_keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*model.cloud_xyzsift, *sift_source_keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud_xyzsift, *sift_target_keypoints);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_rgb_cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*model.cloud_xyzrgb, *source_rgb_cloud_transformed);

	CLOG(LWARNING)<< "SingleS2OMMatcher::prepare common...";

	// prepare common shot + sift source and target clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr common_source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr common_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

	*common_source_keypoints += *shot_source_keypoints;
	*common_source_keypoints += *sift_source_keypoints;

	*common_target_keypoints += *shot_target_keypoints;
	*common_target_keypoints += *sift_target_keypoints;

	pcl::CorrespondencesPtr commonCorrs(new pcl::Correspondences());
	int shotCorrsSize = bestShotCorrs->size();
	for (int i = 0; i < shotCorrsSize; ++i) {
		commonCorrs->push_back(pcl::Correspondence(bestShotCorrs->at(i)));
	}
	for (int i = 0; i < bestSiftCorrs->size(); ++i) {
		pcl::Correspondence old = bestSiftCorrs->at(i);
		commonCorrs->push_back(pcl::Correspondence(old.index_query + shot_source_keypoints->size(), old.index_match + shot_target_keypoints->size(), old.distance));
	}

	CLOG(LWARNING) << "check shot corrs";
	checkCorrespondences(*bestShotCorrs, *shot_source_keypoints, *shot_target_keypoints);
	CLOG(LWARNING) << "check sift corrs";
	checkCorrespondences(*bestSiftCorrs, *sift_source_keypoints, *sift_target_keypoints);
	CLOG(LWARNING) << "check common corrs";
	checkCorrespondences(*commonCorrs, *common_source_keypoints, *common_target_keypoints);

	pcl::transformPointCloud (*shot_source_keypoints, *shot_source_keypoints, shotTransform.getElements());
	pcl::transformPointCloud (*source_rgb_cloud_transformed, *source_rgb_cloud_transformed, shotTransform.getElements());

	out_correspondeces_sift.write(bestSiftCorrs);
	out_correspondeces_sift_trans.write(siftTransform);
	out_correspondeces_sift_source_keypoints.write(sift_source_keypoints);
	out_correspondeces_sift_target_keypoints.write(sift_target_keypoints);

	out_correspondeces_shot.write(bestShotCorrs);
	out_correspondeces_shot_trans.write(shotTransform);
	out_correspondeces_shot_source_keypoints.write(shot_source_keypoints);
	out_correspondeces_shot_target_keypoints.write(shot_target_keypoints);

	out_correspondeces_source_cloud.write(source_rgb_cloud_transformed);
	out_correspondeces_target_cloud.write(cloud_xyzrgb);

	out_correspondeces_common.write(commonCorrs);
	out_correspondeces_common_source_keypoints.write(common_source_keypoints);
	out_correspondeces_common_target_keypoints.write(common_target_keypoints);
}

void SingleS2OMMatcher::checkCorrespondences(pcl::Correspondences corrs, pcl::PointCloud<pcl::PointXYZ> source, pcl::PointCloud<pcl::PointXYZ> target) {
	int source_size = source.size();
	int target_size = target.size();
	for (int i = 0; i < corrs.size(); ++i) {
		pcl::Correspondence corr = corrs[i];
		if (corr.index_query >= source_size) {
			CLOG(LERROR) << "source size : " << source_size << ", index: " << corr.index_query;
		}
		if (corr.index_match >= target_size) {
			CLOG(LERROR) << "target size : " << target_size << ", index: " << corr.index_match;
		}
	}
}

pcl::CorrespondencesPtr SingleS2OMMatcher::computeSHOTCorrespondences(pcl::PointCloud<PointXYZSHOT>::Ptr source,
		pcl::PointCloud<PointXYZSHOT>::Ptr target) {

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	SHOTonlyDescriptorRepresentation::Ptr point_representation(new SHOTonlyDescriptorRepresentation());

	pcl::KdTreeFLANN<PointXYZSHOT> match_search;
	match_search.setPointRepresentation(point_representation);
	match_search.setInputCloud(target);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t j = 0; j < source->size(); ++j) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite (source->at (j).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(source->at(j), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1 && neigh_sqr_dists[0] < SHOT_maxDistance  ) { // && neigh_sqr_dists[0] < 1.0f) { // TODO as property
			pcl::Correspondence correspondence(static_cast<int>(j), neigh_indices[0], neigh_sqr_dists[0]);
			correspondences->push_back(correspondence);
		}
	}

	return correspondences;

}
pcl::CorrespondencesPtr SingleS2OMMatcher::computeSIFTCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr source,
		pcl::PointCloud<PointXYZSIFT>::Ptr target) {

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	SIFTOnlyFeatureRepresentation::Ptr point_representation(new SIFTOnlyFeatureRepresentation());

	pcl::KdTreeFLANN<PointXYZSIFT> match_search;
	match_search.setPointRepresentation(point_representation);
	match_search.setInputCloud(target);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t j = 0; j < source->size(); ++j) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite (source->at (j).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(source->at(j), 1, neigh_indices, neigh_sqr_dists);
		if (found_neighs == 1) {
			pcl::Correspondence correspondence(static_cast<int>(j), neigh_indices[0], neigh_sqr_dists[0]);
			correspondences->push_back(correspondence);
		}
	}

	return correspondences;

}

} //: namespace SingleS2OMMatcher
} //: namespace Processors
