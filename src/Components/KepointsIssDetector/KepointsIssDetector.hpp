/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef KEPOINTSISSDETECTOR_HPP_
#define KEPOINTSISSDETECTOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace KepointsIssDetector {

/*!
 * \class KepointsIssDetector
 * \brief KepointsIssDetector processor class.
 *
 * KepointsIssDetector processor.
 */
class KepointsIssDetector: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	KepointsIssDetector(const std::string & name = "KepointsIssDetector");

	/*!
	 * Destructor
	 */
	virtual ~KepointsIssDetector();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> in_cloud_xyzrgb_normals;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_cloud_xyz;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb_normals;
	Base::DataStreamOut<std::vector<int> > out_indices;

	// Handlers
	Base::EventHandler2 h_compute_xyz;
	Base::EventHandler2 h_compute_xyzrgb;
	Base::EventHandler2 h_compute_xyzrgb_normals;

	Base::Property<double> gamma_21;
	Base::Property<double> gamma_32;
	Base::Property<int> min_neighbors;
	Base::Property<double> model_resolution;
	Base::Property<double> normal_radius;

	void computeXYZ();
	void computeXYZRGB();
	void computeXYZRGBNormals();
};

} //: namespace KepointsIssDetector
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("KepointsIssDetector", Processors::KepointsIssDetector::KepointsIssDetector)

#endif /* KEPOINTSISSDETECTOR_HPP_ */
