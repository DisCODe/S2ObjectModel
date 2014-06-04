/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef S2OBJECTVIEWER_HPP_
#define S2OBJECTVIEWER_HPP_

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

namespace Processors {
namespace S2ObjectViewer {

/*!
 * \class S2ObjectViewer
 * \brief S2ObjectViewer processor class.
 *
 * S2ObjectViewer processor.
 */
class S2ObjectViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	S2ObjectViewer(const std::string & name = "S2ObjectViewer");

	/*!
	 * Destructor
	 */
	virtual ~S2ObjectViewer();

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
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_shots;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_sifts;

	pcl::visualization::PCLVisualizer * viewer;

};

} //: namespace S2ObjectViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2ObjectViewer", Processors::S2ObjectViewer::S2ObjectViewer)

#endif /* S2OBJECTVIEWER_HPP_ */
