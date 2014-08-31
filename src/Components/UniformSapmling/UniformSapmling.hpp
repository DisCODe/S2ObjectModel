/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef UNIFORMSAPMLING_HPP_
#define UNIFORMSAPMLING_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace UniformSapmling {

/*!
 * \class UniformSapmling
 * \brief UniformSapmling processor class.
 *
 * UniformSapmling processor.
 */
class UniformSapmling: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	UniformSapmling(const std::string & name = "UniformSapmling");

	/*!
	 * Destructor
	 */
	virtual ~UniformSapmling();

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

	Base::Property<double> radius_search;

	void computeXYZ();
	void computeXYZRGB();

};

} //: namespace UniformSapmling
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("UniformSapmling", Processors::UniformSapmling::UniformSapmling)

#endif /* UNIFORMSAPMLING_HPP_ */
