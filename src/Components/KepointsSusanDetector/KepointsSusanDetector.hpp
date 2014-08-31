/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef KEPOINTSSUSANDETECTOR_HPP_
#define KEPOINTSSUSANDETECTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace KepointsSusanDetector {

/*!
 * \class KepointsSusanDetector
 * \brief KepointsSusanDetector processor class.
 *
 * KepointsSusanDetector processor.
 */
class KepointsSusanDetector: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KepointsSusanDetector(const std::string & name = "KepointsSusanDetector");

	/*!
	 * Destructor
	 */
	virtual ~KepointsSusanDetector();

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

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<std::vector<int> > out_indices;

	// Handlers
	Base::EventHandler2 h_compute_xyzrgb;

	Base::Property<double> radius;
	Base::Property<double> radius_search;

	void computeXYZRGB();

};

} //: namespace KepointsSusanDetector
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KepointsSusanDetector", Processors::KepointsSusanDetector::KepointsSusanDetector)

#endif /* KEPOINTSSUSANDETECTOR_HPP_ */
