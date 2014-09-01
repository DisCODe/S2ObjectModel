/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef KEYPOINTSVIEWER_HPP_
#define KEYPOINTSVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace KeypointsViewer {

/*!
 * \class KeypointsViewer
 * \brief KeypointsViewer processor class.
 *
 * KeypointsViewer processor.
 */
class KeypointsViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KeypointsViewer(const std::string & name = "KeypointsViewer");

	/*!
	 * Destructor
	 */
	virtual ~KeypointsViewer();

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

	void display();
	void on_spin();

	Base::EventHandler2 h_display;
	Base::EventHandler2 h_on_spin;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_keypoints;
	pcl::visualization::PCLVisualizer * viewer;
	
	int left;
	int right;

};

} //: namespace KeypointsViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KeypointsViewer", Processors::KeypointsViewer::KeypointsViewer)

#endif /* KEYPOINTSVIEWER_HPP_ */
