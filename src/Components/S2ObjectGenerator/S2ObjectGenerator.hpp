/*!
 * \file
 * \brief 
 * \author jkrasnod
 */

#ifndef S2ObjectGenerator_HPP_
#define S2ObjectGenerator_HPP_

#include <Component_Aux.hpp>
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/PointXYZSIFTSHOT.hpp>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::PointCloud<PointXYZSHOT> XYZSHOTCloud;
typedef XYZSHOTCloud::Ptr XYZSHOTCloudPtr;

typedef pcl::PointCloud<PointXYZSIFT> XYZSIFTCloud;
typedef XYZSIFTCloud::Ptr XYZSIFTCloudPtr;

typedef PointXYZSIFTSHOT PointS2;
typedef pcl::PointCloud<PointS2> S2Cloud;
typedef S2Cloud::Ptr S2CloudPtr;


namespace Processors {
namespace S2ObjectGenerator {

/*!
 * \class S2ObjectGenerator
 * \brief S2ObjectGenerator processor class.
 *
 * S2ObjectGenerator processor.
 */
class S2ObjectGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
    S2ObjectGenerator(const std::string & name = "S2ObjectGenerator");

	/*!
	 * Destructor
	 */
    virtual ~S2ObjectGenerator();

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
	Base::DataStreamIn<PointCloudPtr> in_cloud_xyzrgb;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<XYZSIFTCloudPtr> in_cloud_xyzsift;

	/// Input data stream containing feature cloud from a given view.
	Base::DataStreamIn<XYZSHOTCloudPtr> in_cloud_xyzshot;

//	/// Output data stream containing SIFTObjectModel - depricated.
//	Base::DataStreamOut<AbstractObject*> out_instance;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<PointCloudPtr> out_cloud_xyzrgb;

	/// Output data stream containing object model feature cloud (SIFTs).
	Base::DataStreamOut<XYZSIFTCloudPtr> out_cloud_xyzsift;

	/// Output data stream containing object model feature cloud (SHOTs).
	Base::DataStreamOut<XYZSHOTCloudPtr> out_cloud_xyzshot;

	/// Output data stream containing object model feature cloud (SHOTs and SIFTs).
	Base::DataStreamOut<S2CloudPtr> out_cloud_s2;

	// Mean number of features per view.
	//Base::DataStreamOut<int> out_mean_viewpoint_features_number;

	// Handlers
    Base::EventHandler2 h_addViewToModel;
	
	// Handlers
    void addViewToModel();

	/// Computes the transformation between two XYZSIFT clouds basing on the found correspondences.
	Eigen::Matrix4f computeTransformationSAC(const S2Cloud::ConstPtr &cloud_src, const S2Cloud::ConstPtr &cloud_trg,
		const pcl::CorrespondencesConstPtr& correspondences, pcl::Correspondences& inliers);


	/// Number of views.
	int counter;

	/// Total number of features (in all views).
	//int total_viewpoint_features_number;

	PointCloudPtr cloud_merged;
	XYZSIFTCloudPtr cloud_sift_merged;
	XYZSHOTCloudPtr cloud_shot_merged;
	S2CloudPtr cloud_s2_merged;
	Eigen::Matrix4f global_trans;

    /// Alignment mode: use ICP alignment or not.
    Base::Property<bool> prop_ICP_alignment;

};

} //: namespace S2ObjectGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2ObjectGenerator", Processors::S2ObjectGenerator::S2ObjectGenerator)

#endif /* S2ObjectGenerator_HPP_ */
