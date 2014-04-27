/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SUSANKEYPOINTS_HPP_
#define SUSANKEYPOINTS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace SusanKeypoints {

/*!
 * \class SusanKeypoints
 * \brief SusanKeypoints processor class.
 *
 * SusanKeypoints processor.
 */
class SusanKeypoints: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SusanKeypoints(const std::string & name = "SusanKeypoints");

	/*!
	 * Destructor
	 */
	virtual ~SusanKeypoints();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_keypoints;

	// Handlers
    Base::EventHandler2 h_compute;

	// Handlers
    void compute();
	

};

} //: namespace SusanKeypoints
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SusanKeypoints", Processors::SusanKeypoints::SusanKeypoints)

#endif /* SUSANKEYPOINTS_HPP_ */
