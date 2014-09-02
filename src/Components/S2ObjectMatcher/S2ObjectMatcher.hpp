/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef S2OBJECTMATCHER_HPP_
#define S2OBJECTMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>

#include <Types/S2ObjectModel.hpp>
#include <pcl/point_representation.h>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>

namespace Processors {
namespace S2ObjectMatcher {

/*!
 * \class S2ObjectMatcher
 * \brief S2ObjectMatcher processor class.
 *
 * S2ObjectMatcher processor.
 */
class S2ObjectMatcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	S2ObjectMatcher(const std::string & name = "S2ObjectMatcher");

	/*!
	 * Destructor
	 */
	virtual ~S2ObjectMatcher();

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

	// Input data streams

		Base::DataStreamIn<std::vector<AbstractObject*> > in_models;

		Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot;
		Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;
		Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	// Output data streams
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_source_keypoints;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_target_keypoints;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_source;
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_target;
		Base::DataStreamOut<pcl::CorrespondencesPtr> out_correspondences;


		// Handlers
		Base::EventHandler2 h_readModels;
		Base::EventHandler2 h_matchShots;
		Base::EventHandler2 h_matchSifts;


		// Handlers
		void readModels();
		void matchShots();
		void matchSifts();

		std::vector<S2ObjectModel*> models;

};

} //: namespace S2ObjectMatcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2ObjectMatcher", Processors::S2ObjectMatcher::S2ObjectMatcher)

#endif /* S2OBJECTMATCHER_HPP_ */
