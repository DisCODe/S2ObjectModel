/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef KEPOINTSHARRIS3DDETECTOR_HPP_
#define KEPOINTSHARRIS3DDETECTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace KepointsHarris3dDetector {

/*!
 * \class KepointsHarris3dDetector
 * \brief KepointsHarris3dDetector processor class.
 *
 * KepointsHarris3dDetector processor.
 */
class KepointsHarris3dDetector: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KepointsHarris3dDetector(const std::string & name = "KepointsHarris3dDetector");

	/*!
	 * Destructor
	 */
	virtual ~KepointsHarris3dDetector();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<std::vector<int> > out_indices;

	// Handlers
	Base::EventHandler2 h_compute_xyz;
	Base::EventHandler2 h_compute_xyzrgb;

	Base::Property<double> radius;
	Base::Property<double> radius_search;

	void computeXYZ();
	void computeXYZRGB();

};

} //: namespace KepointsHarris3dDetector
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KepointsHarris3dDetector", Processors::KepointsHarris3dDetector::KepointsHarris3dDetector)

#endif /* KEPOINTSHARRIS3DDETECTOR_HPP_ */
