/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SINGLES20MMATCHER_HPP_
#define SINGLES20MMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Types/PointXYZSHOT.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <pcl/correspondence.h>

namespace Processors {
namespace SingleS20MMatcher {

/*!
 * \class SingleS20MMatcher
 * \brief SingleS20MMatcher processor class.
 *
 * SingleS20MMatcher processor.
 */
class SingleS20MMatcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SingleS20MMatcher(const std::string & name = "SingleS20MMatcher");

	/*!
	 * Destructor
	 */
	virtual ~SingleS20MMatcher();

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

	void matchSifts();
	void matchShots();
	void matchAll();
	void refreshModel();

	Base::EventHandler2 h_matchShots;
	Base::EventHandler2 h_matchSifts;
	Base::EventHandler2 h_matchAll;
	Base::EventHandler2 h_refreshModel;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_model_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_model_xyzshot;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_model_xyzsift;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_xyzshot;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_xyzsift;

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_source_keypoints_xyzshot;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_target_keypoints_xyzshot;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_target_keypoints_xyzsift;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_source_keypoints_xyzsift;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_source;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_target;
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences_sift;
	Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences_shot;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb;
	pcl::PointCloud<PointXYZSHOT>::Ptr shots;
	pcl::PointCloud<PointXYZSIFT>::Ptr sifts;

	//Base::Property<bool> use_shots;
};

} //: namespace SingleS20MMatcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SingleS20MMatcher", Processors::SingleS20MMatcher::SingleS20MMatcher)

#endif /* SINGLES20MMATCHER_HPP_ */
