/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>
#include <iostream>
#include <set>

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
#include <pcl/registration/correspondence_rejection_one_to_one.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Types/Features.hpp"
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
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

Types::HomogMatrix getBestCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::CorrespondencesPtr correspondences,
		pcl::CorrespondencesPtr bestCorrespondences, double inlierThreshold, int maxIter);

void removeInCenter(pcl::PointCloud<PointXYZSIFT>::Ptr cloud, pcl::PointCloud<PointXYZSIFT>::Ptr cloud_filtered);

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
		Base::Component(name), RANSAC_MaximumIterations("ransac.max_iterations", 100), RANSAC_InlierThreshold(
				"ransac.inlier_threshold", 0.5), shots_useReciprocalCorrespondeces("shot.oneToOne", 1), sifts_useReciprocalCorrespondeces(
				"sift.oneToOne", 1), shots_useReciprocalCorrespondecesNextBest("shot.nextBest", 0), sifts_useReciprocalCorrespondecesNextBest(
				"sift.nextBest", 0), SHOT_maxDistance("shot.max_disance", 2), SIFT_maxDistance("sift.max_disance",
				100000) {

	registerProperty(RANSAC_MaximumIterations);
	registerProperty(RANSAC_InlierThreshold);
	registerProperty(SHOT_maxDistance);
	registerProperty(SIFT_maxDistance);
	registerProperty(shots_useReciprocalCorrespondeces);
	registerProperty(sifts_useReciprocalCorrespondeces);
	registerProperty(shots_useReciprocalCorrespondecesNextBest);
	registerProperty(sifts_useReciprocalCorrespondecesNextBest);
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
	registerStream("out_matrix_sift", &out_matrix_sift);
	registerStream("out_source_keypoints_sift", &out_source_keypoints_sift);
	registerStream("out_target_keypoints_sift", &out_target_keypoints_sift);

	registerStream("out_correspondeces_shot", &out_correspondeces_shot);
	registerStream("out_matrix_shot", &out_matrix_shot);
	registerStream("out_source_keypoints_shot", &out_source_keypoints_shot);
	registerStream("out_target_keypoints_shot", &out_target_keypoints_shot);

	registerStream("out_correspondeces_common_ransac", &out_correspondeces_common_ransac);
	registerStream("out_matrix_common_ransac", &out_matrix_common_ransac);
	registerStream("out_source_keypoints_common_ransac", &out_source_keypoints_common_ransac);
	registerStream("out_target_keypoints_common_ransac", &out_target_keypoints_common_ransac);

	registerStream("out_correspondeces_shot_sift", &out_correspondeces_shot_sift);

	registerStream("out_source_cloud", &out_source_cloud);
	registerStream("out_target_cloud", &out_target_cloud);

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

	pcl::PointCloud<PointXYZSIFT>::Ptr sift_fixed (new pcl::PointCloud<PointXYZSIFT>());
	removeInCenter(cloud_xyzsift, sift_fixed);
	CLOG(LWARNING) << "SingleS2OMMatcher::match - fix sifts; before : " << cloud_xyzsift->size() << ", after : " << sift_fixed->size();

	for (int i = 0; i < models.size(); ++i) {
		matchModel(*models[i], cloud_xyzrgb, sift_fixed, cloud_xyzshot);
	}
	return;
}

void SingleS2OMMatcher::matchModel(S2ObjectModel model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb,
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift, pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot) {
	CLOG(LWARNING)<< "SingleS2OMMatcher::matchModel " << model.name;

	// prepare xyz clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr shot_source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*model.cloud_xyzshot, *shot_source_keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr shot_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud_xyzshot, *shot_target_keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*model.cloud_xyzsift, *sift_source_keypoints);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sift_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud_xyzsift, *sift_target_keypoints);

	// prepare common shot + sift source and target clouds
	pcl::PointCloud<pcl::PointXYZ>::Ptr common_source_keypoints(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr common_target_keypoints(new pcl::PointCloud<pcl::PointXYZ>());

	*common_source_keypoints += *shot_source_keypoints;
	*common_source_keypoints += *sift_source_keypoints;

	*common_target_keypoints += *shot_target_keypoints;
	*common_target_keypoints += *sift_target_keypoints;

	Types::HomogMatrix shotTransform;
	Types::HomogMatrix siftTransform;
	Types::HomogMatrix commonTransform;

	pcl::CorrespondencesPtr bestSiftCorrs(new pcl::Correspondences());
	pcl::CorrespondencesPtr bestShotCorrs(new pcl::Correspondences());
	pcl::CorrespondencesPtr bestCommonCorrs(new pcl::Correspondences());

	pcl::CorrespondencesPtr shotCorrs = computeSHOTCorrespondences(model.cloud_xyzshot, cloud_xyzshot);
	pcl::CorrespondencesPtr siftCorrs = computeSIFTCorrespondences(model.cloud_xyzsift, cloud_xyzsift);

	pcl::CorrespondencesPtr commonCorrs(new pcl::Correspondences());

	CLOG(LWARNING) << "Common source keypoints: " << shot_source_keypoints->size() << " + " <<
			sift_source_keypoints->size() << " = " << common_source_keypoints->size();
	CLOG(LWARNING) << "Common target keypoints: " << shot_target_keypoints->size() << " + " <<
			sift_target_keypoints->size() << " = " << common_target_keypoints->size();

	for (int i = 0; i < shotCorrs->size(); ++i) {
		commonCorrs->push_back(pcl::Correspondence(shotCorrs->at(i)));
	}
	for (int i = 0; i < siftCorrs->size(); ++i) {
		pcl::Correspondence old = siftCorrs->at(i);
		commonCorrs->push_back(pcl::Correspondence(old.index_query + shot_source_keypoints->size(), old.index_match + shot_target_keypoints->size(), old.distance));
	}

	// RANSAC

	shotTransform = getBestCorrespondences(shot_source_keypoints, shot_target_keypoints,shotCorrs, bestShotCorrs,
			RANSAC_InlierThreshold, RANSAC_MaximumIterations);
	siftTransform = getBestCorrespondences(sift_source_keypoints, sift_target_keypoints,siftCorrs, bestSiftCorrs,
			RANSAC_InlierThreshold, RANSAC_MaximumIterations);
	commonTransform = getBestCorrespondences(common_source_keypoints, common_target_keypoints, commonCorrs, bestCommonCorrs,
			RANSAC_InlierThreshold, RANSAC_MaximumIterations);

	// prepare shotSiftCorrespondeces

	pcl::CorrespondencesPtr shotSiftCorrs(new pcl::Correspondences());

	for (int i = 0; i < bestShotCorrs->size(); ++i) {
		shotSiftCorrs->push_back(pcl::Correspondence(bestShotCorrs->at(i)));
	}
	for (int i = 0; i < bestSiftCorrs->size(); ++i) {
		pcl::Correspondence old = bestSiftCorrs->at(i);
		shotSiftCorrs->push_back(pcl::Correspondence(old.index_query + shot_source_keypoints->size(), old.index_match + shot_target_keypoints->size(), old.distance));
	}

	CLOG(LWARNING)<< "SingleS2OMMatcher::shotCorrs " << shotCorrs->size();
	 CLOG(LWARNING)<< "SingleS2OMMatcher::bestShotCorrs " << bestShotCorrs->size();
	 CLOG(LWARNING)<< "SingleS2OMMatcher::siftCorrs " << siftCorrs->size();
	 CLOG(LWARNING)<< "SingleS2OMMatcher::bestSiftCorrs " << bestSiftCorrs->size();
	 CLOG(LWARNING)<< "SingleS2OMMatcher::commonCorrs " << commonCorrs->size();
	 CLOG(LWARNING)<< "SingleS2OMMatcher::bestCommonCorrs " << bestCommonCorrs->size();
	 CLOG(LWARNING)<< "SingleS2OMMatcher::shotSiftCorrs " << shotSiftCorrs->size();

	CLOG(LWARNING) << "check shot corrs";
	checkCorrespondences(*bestShotCorrs, *shot_source_keypoints, *shot_target_keypoints);
	CLOG(LWARNING) << "check sift corrs";
	checkCorrespondences(*bestSiftCorrs, *sift_source_keypoints, *sift_target_keypoints);
	CLOG(LWARNING) << "check common corrs";
	checkCorrespondences(*bestCommonCorrs, *common_source_keypoints, *common_target_keypoints);
	CLOG(LWARNING) << "check shot sift corrs";
	checkCorrespondences(*shotSiftCorrs, *common_source_keypoints, *common_target_keypoints);

	out_correspondeces_sift.write(bestSiftCorrs);
	CLOG(LWARNING) << "out_matrix_sift.write\n" << siftTransform.getElements();
	out_matrix_sift.write(siftTransform);
	out_source_keypoints_sift.write(sift_source_keypoints);
	out_target_keypoints_sift.write(sift_target_keypoints);

	out_correspondeces_shot.write(bestShotCorrs);
	CLOG(LWARNING) << "out_matrix_shot.write\n" << shotTransform.getElements();
	out_matrix_shot.write(shotTransform);
	out_source_keypoints_shot.write(shot_source_keypoints);
	out_target_keypoints_shot.write(shot_target_keypoints);

	out_correspondeces_common_ransac.write(bestCommonCorrs);
	CLOG(LWARNING) << "out_matrix_common_ransac.write\n" << commonTransform.getElements();
	out_matrix_common_ransac.write(commonTransform);
	out_source_keypoints_common_ransac.write(common_source_keypoints);
	out_target_keypoints_common_ransac.write(common_target_keypoints);

	out_correspondeces_shot_sift.write(shotSiftCorrs);

	out_source_cloud.write(model.cloud_xyzrgb);
	out_target_cloud.write(cloud_xyzrgb);
}

void SingleS2OMMatcher::checkCorrespondences(pcl::Correspondences corrs, pcl::PointCloud<pcl::PointXYZ> source,
		pcl::PointCloud<pcl::PointXYZ> target) {
	int source_size = source.size();
	int target_size = target.size();
	for (int i = 0; i < corrs.size(); ++i) {
		pcl::Correspondence corr = corrs[i];
		if (corr.index_query >= source_size) {
			CLOG(LERROR)<< "source size : " << source_size << ", index: " << corr.index_query;
		}
		if (corr.index_match >= target_size) {
			CLOG(LERROR)<< "target size : " << target_size << ", index: " << corr.index_match;
		}
	}
}

pcl::CorrespondencesPtr SingleS2OMMatcher::computeSHOTCorrespondences(pcl::PointCloud<PointXYZSHOT>::Ptr source,
		pcl::PointCloud<PointXYZSHOT>::Ptr target) {

	std::set<int> used;
	bool useNextNeigh = shots_useReciprocalCorrespondecesNextBest > 0;
	int k = shots_useReciprocalCorrespondecesNextBest == 0 ? 1 : shots_useReciprocalCorrespondecesNextBest;

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
	SHOTonlyDescriptorRepresentation::Ptr point_representation(new SHOTonlyDescriptorRepresentation());

	pcl::KdTreeFLANN<PointXYZSHOT> match_search;
	match_search.setPointRepresentation(point_representation);
	match_search.setInputCloud(target);

	for (size_t j = 0; j < source->size(); ++j) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite (source->at (j).descriptor[0]))
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(source->at(j), k, neigh_indices, neigh_sqr_dists);

		for (int i = 0; i < found_neighs; ++i) {
			float dist = neigh_sqr_dists[i];
			int target_index = neigh_indices[i];
			if (dist <= SHOT_maxDistance) {
				if (used.find(target_index) == used.end()) {
					pcl::Correspondence correspondence(static_cast<int>(j), target_index, dist);
					correspondences->push_back(correspondence);
					if (useNextNeigh)
						used.insert(target_index);
					break;
				}
			} else {
				break;
			}

		}

	}

	if (shots_useReciprocalCorrespondeces) {
		pcl::registration::CorrespondenceRejectorOneToOne rejec;
		rejec.getRemainingCorrespondences(*correspondences, *correspondences);
	}

	return correspondences;

}
pcl::CorrespondencesPtr SingleS2OMMatcher::computeSIFTCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr source,
		pcl::PointCloud<PointXYZSIFT>::Ptr target) {

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

	SIFTOnlyFeatureRepresentation::Ptr point_representation(new SIFTOnlyFeatureRepresentation());

	std::set<int> used;
	bool useNextNeigh = sifts_useReciprocalCorrespondecesNextBest > 0;
	int k = sifts_useReciprocalCorrespondecesNextBest == 0 ? 1 : sifts_useReciprocalCorrespondecesNextBest;

	pcl::KdTreeFLANN<PointXYZSIFT> match_search;
	match_search.setPointRepresentation(point_representation);
	match_search.setInputCloud(target);

	for (size_t j = 0; j < source->size(); ++j) {
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite (source->at (j).descriptor[0]))
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(source->at(j), k, neigh_indices, neigh_sqr_dists);

		for (int i = 0; i < found_neighs; ++i) {
			float dist = neigh_sqr_dists[i];
			int target_index = neigh_indices[i];
			if (dist <= SIFT_maxDistance) {
				if (used.find(target_index) == used.end()) {
					pcl::Correspondence correspondence(static_cast<int>(j), target_index, dist);
					correspondences->push_back(correspondence);
					if (useNextNeigh)
						used.insert(target_index);
					break;
				}
			} else {
				break;
			}

		}

	}

	if (sifts_useReciprocalCorrespondeces) {
		pcl::registration::CorrespondenceRejectorOneToOne rejec;
		rejec.getRemainingCorrespondences(*correspondences, *correspondences);
	}

	return correspondences;
}

void removeInCenter(pcl::PointCloud<PointXYZSIFT>::Ptr cloud, pcl::PointCloud<PointXYZSIFT>::Ptr cloud_filtered) {

	pcl::copyPointCloud(*cloud, *cloud_filtered);
	pcl::PointCloud<PointXYZSIFT>::iterator it = cloud_filtered->begin();
	while (it != cloud_filtered->end()) {
		if (it->x == 0 && it->y == 0 && it->z == 0) {
			it = cloud_filtered->erase(it);
		} else {
			++it;
		}
	}
}

Types::HomogMatrix getBestCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
		pcl::PointCloud<pcl::PointXYZ>::Ptr target, pcl::CorrespondencesPtr correspondences,
		pcl::CorrespondencesPtr bestCorrespondences, double inlierThreshold, int maxIter) {

	Types::HomogMatrix transform;
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> ransac;
	ransac.setInputSource(source);
	ransac.setInputTarget(target);
	ransac.setInlierThreshold(inlierThreshold);
	ransac.setMaximumIterations(maxIter);
	ransac.setInputCorrespondences(correspondences);
	ransac.getCorrespondences(*bestCorrespondences);

	Eigen::Matrix4f temp = ransac.getBestTransformation();
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			transform.setElement(i, j, (float) temp(i, j));
		}
	}
	return transform;
}

} //: namespace SingleS2OMMatcher
} //: namespace Processors
