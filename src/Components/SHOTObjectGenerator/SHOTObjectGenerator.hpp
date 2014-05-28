/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SHOTOBJECTGENERATOR_HPP_
#define SHOTOBJECTGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
#include <Types/PointXYZSHOT.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

namespace Processors {
namespace SHOTObjectGenerator {

/*!
 * \class SHOTObjectGenerator
 * \brief SHOTObjectGenerator processor class.
 *
 * SHOTObjectGenerator processor.
 */
class SHOTObjectGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOTObjectGenerator(const std::string & name = "SHOTObjectGenerator");

	/*!
	 * Destructor
	 */
	virtual ~SHOTObjectGenerator();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	/// Input data stream containing point cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;

	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_cloud_xyzshot;

	// Mean number of features per view.
	Base::DataStreamOut<int> out_mean_viewpoint_features_number;

	// Handlers
	Base::EventHandler2 h_addViewToModel;

	// Handlers
	void addViewToModel();

	/// Computes the transformation between two XYZSHOT clouds basing on the found correspondences.
	Eigen::Matrix4f computeTransformationSAC(const pcl::PointCloud<PointXYZSHOT>::ConstPtr &cloud_src,
			const pcl::PointCloud<PointXYZSHOT>::ConstPtr &cloud_trg,
			const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers);

	/// Number of views.
	int counter;

	/// Total number of features (in all views).
	int total_viewpoint_features_number;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merged;
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_shot_merged;
	Eigen::Matrix4f global_trans;

	/// Alignment mode: use ICP alignment or not.
	Base::Property<bool> prop_ICP_alignment;
};

} //: namespace SHOTObjectGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOTObjectGenerator", Processors::SHOTObjectGenerator::SHOTObjectGenerator)

#endif /* SHOTOBJECTGENERATOR_HPP_ */
