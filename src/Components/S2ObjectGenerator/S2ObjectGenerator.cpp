/*!
 * \file
 * \brief
 * \author jkrasnod
 */

#include <memory>
#include <string>

#include<iostream>

#include "S2ObjectGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/impl/instantiate.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/filter.h>

#include<pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

namespace Processors {
namespace S2ObjectGenerator {

class S2Representation: public pcl::DefaultFeatureRepresentation<PointS2> {
	using pcl::PointRepresentation<PointS2>::nr_dimensions_;

public:
	S2Representation() {
		nr_dimensions_ = 128 + 352;
		trivial_ = false;
	}

	virtual void copyToFloatArray(const PointS2 &p, float * out) const {
		for (register int i = 0; i < 128 + 352; i++) {
			out[i] = p.sift[i];
			if (i < 128) {
				out[i] = p.sift[i];
			} else {
				out[i] = p.shot[i - 128];
			}
		}
	}
};

class SIFTRepresentation: public pcl::DefaultFeatureRepresentation<PointS2> {
	using pcl::PointRepresentation<PointS2>::nr_dimensions_;

public:
	SIFTRepresentation() {

		nr_dimensions_ = 128;
		trivial_ = false;
	}

	virtual void copyToFloatArray(const PointS2 &p, float * out) const {
		for (register int i = 0; i < 128; i++) {
			out[i] = p.sift[i];
		}
	}
};

class SHOTRepresentation: public pcl::DefaultFeatureRepresentation<PointS2> {
	using pcl::PointRepresentation<PointS2>::nr_dimensions_;

public:
	SHOTRepresentation() {
		nr_dimensions_ = 352;
		trivial_ = false;
	}

	virtual void copyToFloatArray(const PointS2 &p, float * out) const {
		for (register int i = 0; i < 352; i++) {
			out[i] = p.shot[i];
		}
	}
};

/*
 * \brief Class used for transformation from SIFT descriptor to array of floats.
 */
class SIFTFeatureRepresentation: public pcl::DefaultFeatureRepresentation<
		PointXYZSIFT> //could possibly be pcl::PointRepresentation<...> ??
{
	/// Templatiated number of SIFT descriptor dimensions.
	using pcl::PointRepresentation<PointXYZSIFT>::nr_dimensions_;

public:
	SIFTFeatureRepresentation() {
		// Define the number of dimensions.
		nr_dimensions_ = 128;
		trivial_ = false;
	}

	/// Overrides the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray(const PointXYZSIFT &p, float * out) const {
		//This representation is only for determining correspondences (not for use in Kd-tree for example - so use only the SIFT part of the point.
		for (register int i = 0; i < 128; i++)
			out[i] = p.descriptor[i]; //p.descriptor.at<float>(0, i) ;
	}
};

Eigen::Matrix4f S2ObjectGenerator::computeTransformationSAC(
		const S2Cloud::ConstPtr &cloud_src, const S2Cloud::ConstPtr &cloud_trg,
		const pcl::CorrespondencesConstPtr& correspondences,
		pcl::Correspondences& inliers) {
	CLOG(LTRACE)<< "Computing SAC" << std::endl;

	pcl::registration::CorrespondenceRejectorSampleConsensus<PointS2> sac;
	sac.setInputSource(cloud_src);
	sac.setInputTarget(cloud_trg);
	sac.setInlierThreshold(0.001f);
	sac.setMaximumIterations(2000);
	sac.setInputCorrespondences(correspondences);
	sac.getCorrespondences(inliers);

	CLOG(LINFO) << "SAC inliers " << inliers.size();

	if ( ((float)inliers.size()/(float)correspondences->size()) >85)
	return Eigen::Matrix4f::Identity();
	return sac.getBestTransformation();
}

S2ObjectGenerator::S2ObjectGenerator(const std::string & name) :
		Base::Component(name), prop_ICP_alignment("alignment.use ICP", false) {
	registerProperty(prop_ICP_alignment);

	counter = 0;
}

S2ObjectGenerator::~S2ObjectGenerator() {
}

void S2ObjectGenerator::prepareInterface() {
	// Register data streams.
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
//	registerStream("out_instance", &out_instance);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);
	registerStream("out_cloud_s2", &out_cloud_s2);
	//registerStream("out_mean_viewpoint_features_number", &out_mean_viewpoint_features_number);

	// Register single handler - the "addViewToModel" function.
	h_addViewToModel.setup(
			boost::bind(&S2ObjectGenerator::addViewToModel, this));
	registerHandler("addViewToModel", &h_addViewToModel);
	addDependency("addViewToModel", &in_cloud_xyzsift);
	addDependency("addViewToModel", &in_cloud_xyzshot);
	addDependency("addViewToModel", &in_cloud_xyzrgb);

}

bool S2ObjectGenerator::onInit() {
	// Number of viewpoints.
	counter = 0;
	// Mean number of features per view.
//	mean_viewpoint_features_number = 0;

	global_trans = Eigen::Matrix4f::Identity();

	cloud_merged = PointCloudPtr(new PointCloud());
	cloud_sift_merged = XYZSIFTCloudPtr(new XYZSIFTCloud());
	cloud_shot_merged = XYZSHOTCloudPtr(new XYZSHOTCloud());
	cloud_s2_merged = S2CloudPtr(new S2Cloud());

	return true;
}

bool S2ObjectGenerator::onFinish() {
	return true;
}

bool S2ObjectGenerator::onStop() {
	return true;
}

bool S2ObjectGenerator::onStart() {
	return true;
}

float computeDistance(PointS2 p1, PointS2 p2) {

	float distance = 0;
	for (int i = 0; i < 128; ++i) distance += pow((p1.sift[i] - p2.sift[i]), 2);
	for (int i = 0; i < 352; ++i) distance += pow((p1.shot[i] - p2.shot[i]), 2);
	return distance;
}

S2CloudPtr getPointXYZSIFTSHOTCloud(XYZSIFTCloudPtr sifts, XYZSHOTCloudPtr shots) {

	S2CloudPtr result(new S2Cloud());

	for (int i = 0; i < sifts->size(); ++i) {

		if (sifts->points[i].x != shots->points[i].x
				|| sifts->points[i].y != shots->points[i].y
				|| sifts->points[i].z != shots->points[i].z) {
			std::cout << "SIFTS: " << sifts->points[i].x << "\t"
					<< sifts->points[i].y << "\t" << sifts->points[i].z << "\n";
			std::cout << "SHOTS: " << shots->points[i].x << "\t"
					<< shots->points[i].y << "\t" << shots->points[i].z << "\n";

		}

		PointS2 newPoint;
		newPoint.x = sifts->points[i].x;
		newPoint.y = sifts->points[i].y;
		newPoint.z = sifts->points[i].z;
		for (int j = 0; j < 128; ++j) {
			newPoint.sift[j] = sifts->points[i].descriptor[j];
		}
		for (int j = 0; j < 9; ++j) {
			newPoint.rf[j] = shots->points[i].rf[j];
		}
		for (int j = 0; j < 352; ++j) {
			newPoint.shot[j] = shots->points[i].descriptor[j];
		}
		result->push_back(newPoint);
	}

	return result;
}

void S2ObjectGenerator::addViewToModel() {
	CLOG(LINFO)<< "S2ObjectGenerator::addViewToModel";

	PointCloudPtr cloud = in_cloud_xyzrgb.read();
	XYZSIFTCloudPtr cloud_sift = in_cloud_xyzsift.read();
	XYZSHOTCloudPtr cloud_shot = in_cloud_xyzshot.read();

	// TODO if empty()

	CLOG(LINFO) << "cloud_xyzrgb size: "<<cloud->size();
	CLOG(LINFO) << "cloud_xyzsift size: "<<cloud_sift->size();
	CLOG(LINFO) << "cloud_xyzsift size: "<<cloud_shot->size();

	// Remove NaNs.
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	cloud_sift->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_sift, *cloud_sift, indices);
	cloud_shot->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_shot, *cloud_shot, indices);

	S2CloudPtr cloud_s2 = getPointXYZSIFTSHOTCloud(cloud_sift, cloud_shot);

	CLOG(LDEBUG) << "cloud_xyzrgb size without NaN: "<<cloud->size();
	CLOG(LDEBUG) << "cloud_xyzsift size without NaN: "<<cloud_sift->size();
	CLOG(LDEBUG) << "cloud_xyzshot size without NaN: "<<cloud_shot->size();

	CLOG(LINFO) << "view number: "<<counter;
	CLOG(LINFO) << "view cloud->size(): "<<cloud->size();
	CLOG(LINFO) << "view cloud_sift->size(): "<<cloud_sift->size();
	CLOG(LINFO) << "view cloud_shot->size(): "<<cloud_shot->size();
	CLOG(LINFO) << "view s2cloud->size(): "<<cloud_s2->size();

	// First cloud.
	if (counter == 0 ) {
		*cloud_merged = *cloud;
		*cloud_sift_merged = *cloud_sift;
		*cloud_s2_merged = *cloud_s2;

		counter++;
		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);

		return;
	}

	// S2ObjectGenerator view count and feature numbers.
	counter++;

	// Find corespondences between clouds

	pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
	pcl::registration::CorrespondenceEstimation<PointS2, PointS2> correst;
	S2Representation::Ptr point_representation(new S2Representation());
	correst.setPointRepresentation(point_representation);
	correst.setInputSource(cloud_s2);
	correst.setInputTarget(cloud_s2_merged);
	// Find correspondences.
	correst.determineReciprocalCorrespondences(*correspondences);
	CLOG(LINFO) << "Number of reciprocal correspondences(s2): " << correspondences->size() << " out of " << cloud_sift->size() << " features";

	return;

	/*
	 pcl::CorrespondencesPtr correspondences_sift(new pcl::Correspondences()) ;
	 pcl::registration::CorrespondenceEstimation<PointXYZSIFT, PointXYZSIFT> correst_sift;
	 SIFTFeatureRepresentation::Ptr point_representation(new SIFTFeatureRepresentation()) ;
	 correst_sift.setPointRepresentation(point_representation) ;
	 correst_sift.setInputSource(cloud_sift) ;
	 correst_sift.setInputTarget(cloud_sift_merged);
	 // Find correspondences.
	 correst_sift.determineReciprocalCorrespondences(*correspondences_sift) ;
	 CLOG(LINFO) << "Number of reciprocal correspondences(sift): " << correspondences_sift->size() << " out of " << cloud_sift->size() << " features";

	 // Find corespondences between shot clouds.
	 // Initialize parameters.
	 pcl::CorrespondencesPtr correspondences_shot(new pcl::Correspondences()) ;
	 pcl::registration::CorrespondenceEstimation<PointXYZSHOT, PointXYZSHOT> correst_shot;
	 correst_shot.setInputSource(cloud_shot) ;
	 correst_shot.setInputTarget(cloud_shot_merged) ;
	 // Find correspondences.
	 correst_shot.determineReciprocalCorrespondences(*correspondences_shot) ;
	 CLOG(LINFO) << "Number of reciprocal correspondences(shot): " << correspondences_shot->size() << " out of " << cloud_shot->size() << " features";

	 // Computate multiplicity of features (designating how many multiplicity given feature appears in all views).	for(int i = 0; i< correspondences->size();i++){
	 if (correspondences->at(i).index_query >=cloud_sift->size() || correspondences->at(i).index_match >=cloud_sift_merged->size()){
	 continue;
	 }
	 //
	 cloud_sift->at(correspondences->at(i).index_query).multiplicity = cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity + 1;
	 cloud_sift_merged->at(correspondences->at(i).index_match).multiplicity=-1;
	 }

	//displayCorrespondences(cloud_next, cloud_sift_next, cloud_prev, cloud_sift_prev, correspondences, viewer) ;
	// set distance in s2
	pcl::CorrespondencesPtr s2correspondences = correspondences;

	// Compute transformation between clouds and S2ObjectGenerator global transformation of cloud.
	pcl::Correspondences inliers;
	Eigen::Matrix4f current_trans = computeTransformationSAC(cloud_s2, cloud_s2_merged, correspondences, inliers);
	if (current_trans == Eigen::Matrix4f::Identity()) {
		out_cloud_xyzrgb.write(cloud_merged);
		out_cloud_xyzsift.write(cloud_sift_merged);
		return;
	} //: if
	CLOG(LINFO) << "SAC Transformation from current view cloud to cloud_merged: " << std::endl << current_trans;
		global_trans = global_trans * current_trans;
	 CLOG(LINFO) << "Transformation from current view cloud to first view: " << std::endl << global_trans << std::endl ;


	// Delete points.
	XYZSIFTCloud::iterator pt_iter = cloud_sift_merged->begin();
	while(pt_iter!=cloud_sift_merged->end()) {
		if(pt_iter->multiplicity==-1) {
			pt_iter = cloud_sift_merged->erase(pt_iter);
		} else {
			++pt_iter;
		}
	}

	// Transform both view clouds.
		PointCloudPtr cloud_to_merge;
	 XYZSIFTCloudPtr cloud_sift_to_merge;
	 cloud_to_merge = PointCloudPtr (new PointCloud());
	 cloud_sift_to_merge = XYZSIFTCloudPtr (new XYZSIFTCloud());
	cloud_sift_to_merge = XYZSIFTCloudPtr (new XYZSIFTCloud());

	pcl::transformPointCloud(*cloud, *cloud, current_trans);
	pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

	if (prop_ICP_alignment) {
		// Use ICP to get "better" transformation.
		pcl::IterativeClosestPoint<PointT, PointT> icp;

		//	max_correspondence_distance_ (1.0*1.0),
		//	nr_iterations_ (500)
		//	normal_radius_ (0.2),
		//	feature_radius_ (0.2)

		// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
		icp.setMaxCorrespondenceDistance (0.005);
		// Set the maximum number of iterations (criterion 1)
		icp.setMaximumIterations (50);
		// Set the transformation epsilon (criterion 2)
		icp.setTransformationEpsilon (1e-8);
		// Set the euclidean distance difference epsilon (criterion 3)
		icp.setEuclideanFitnessEpsilon (1);

		icp.setInputSource(cloud_merged);
		icp.setInputTarget(cloud);
		PointCloudPtr Final (new PointCloud());
		icp.align(*Final);
		CLOG(LINFO) << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore();

		// Get the transformation from target to source.
		current_trans = icp.getFinalTransformation().inverse();
		CLOG(LINFO) << "ICP transformation refinement: " << std::endl << current_trans;

		// Refine the transformation.
		pcl::transformPointCloud(*cloud, *cloud, current_trans);
		pcl::transformPointCloud(*cloud_sift, *cloud_sift, current_trans);

	}    //: ICP alignment

	 Eigen::Matrix4f icp_trans;
	 pairAlign (cloud_merged, cloud, icp_trans, false);
	 CLOG(LINFO) << "ICP transformation refinement: " << std::endl << icp_trans;

	//addCloudToScene(cloud_to_merge, sceneviewer, counter - 1) ;

	// Add clouds.
	*cloud_merged += *cloud;
	*cloud_sift_merged += *cloud_sift;

	CLOG(LINFO) << "model cloud->size(): "<<cloud_merged->size();
	CLOG(LINFO) << "model cloud_sift->size(): "<<cloud_sift_merged->size();

	// Compute mean number of features.
//	mean_viewpoint_features_number = total_viewpoint_features_number/counter;

	// Push results to output data ports.
	//out_mean_viewpoint_features_number.write(mean_viewpoint_features_number);
	out_cloud_xyzrgb.write(cloud_merged);
	out_cloud_xyzsift.write(cloud_sift_merged);
*/
	// Push SOM - depricated.
//	out_instance.write(produce());
}

} //: namespace S2ObjectGenerator
} //: namespace Processors
