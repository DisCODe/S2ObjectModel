/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SINGLES2OMMATCHER_HPP_
#define SINGLES2OMMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Types/PointXYZSHOT.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/S2ObjectModel.hpp>
#include <Types/HomogMatrix.hpp>
#include <pcl/point_representation.h>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>


namespace Processors {
namespace SingleS2OMMatcher {

/*!
 * \class SingleS2OMMatcher
 * \brief SingleS2OMMatcher processor class.
 *
 * SingleS2OMMatcher processor.
 */
class SingleS2OMMatcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SingleS2OMMatcher(const std::string & name = "SingleS2OMMatcher");

	/*!
	 * Destructor
	 */
	virtual ~SingleS2OMMatcher();

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

	Base::DataStreamIn<std::vector<AbstractObject*> > in_models;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;

	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondeces_sift;
	Base::DataStreamOut<Types::HomogMatrix> out_matrix_sift;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_source_keypoints_sift;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_target_keypoints_sift;

	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondeces_shot;
	Base::DataStreamOut<Types::HomogMatrix> out_matrix_shot;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_source_keypoints_shot;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_target_keypoints_shot;

	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondeces_common_ransac;
	Base::DataStreamOut<Types::HomogMatrix> out_matrix_common_ransac;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_source_keypoints_common_ransac;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_target_keypoints_common_ransac;

	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondeces_shot_sift;

	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_source_cloud;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_target_cloud;

	//
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb1;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot1;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift1;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb2;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot2;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift2;





	Base::EventHandler2 h_readModels;
	Base::EventHandler2 h_match;

	Base::Property<int> RANSAC_MaximumIterations;
	Base::Property<double> RANSAC_InlierThreshold;
	Base::Property<double> SHOT_maxDistance;
	Base::Property<long> SIFT_maxDistance;

    Base::Property<bool> shots_useReciprocalCorrespondeces;
    Base::Property<bool> sifts_useReciprocalCorrespondeces;
    Base::Property<int> shots_useReciprocalCorrespondecesNextBest;
    Base::Property<int> sifts_useReciprocalCorrespondecesNextBest;

	void readModels();
	void matchPair();
	void match();


	void checkCorrespondences(pcl::Correspondences corrs, pcl::PointCloud<pcl::PointXYZ> source, pcl::PointCloud<pcl::PointXYZ> target);


	std::vector<S2ObjectModel*> models; // size = 1

private:
	void matchModel(S2ObjectModel model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb,
			pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift,
			pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot);
	pcl::CorrespondencesPtr computeSHOTCorrespondences(pcl::PointCloud<PointXYZSHOT>::Ptr source, pcl::PointCloud<PointXYZSHOT>::Ptr target);
	pcl::CorrespondencesPtr computeSIFTCorrespondences(pcl::PointCloud<PointXYZSIFT>::Ptr source, pcl::PointCloud<PointXYZSIFT>::Ptr target);

	void match2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_xyzrgb,
	pcl::PointCloud<PointXYZSIFT>::Ptr model_xyzsift,
	pcl::PointCloud<PointXYZSHOT>::Ptr model_xyzshot,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_xyzrgb,
	pcl::PointCloud<PointXYZSIFT>::Ptr scene_xyzsift,
	pcl::PointCloud<PointXYZSHOT>::Ptr scene_xyzshot);


	// TODO add transformation to compute error
};

} //: namespace SingleS2OMMatcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SingleS2OMMatcher", Processors::SingleS2OMMatcher::SingleS2OMMatcher)

#endif /* SINGLES2OMMATCHER_HPP_ */
