/*!
 * \file
 * \brief 
 * \author jkrasnod
 */

#ifndef CUBOIDNORMALS_HPP_
#define CUBOIDNORMALS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace Processors {
namespace CuboidNormals {

/*!
 * \class CuboidNormals
 * \brief CuboidNormals processor class.
 *
 * CuboidNormals processor.
 */
class CuboidNormals: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CuboidNormals(const std::string & name = "CuboidNormals");

	/*!
	 * Destructor
	 */
	virtual ~CuboidNormals();

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

	// Handlers
	Base::EventHandler2 h_compute_xyz;
	Base::EventHandler2 h_compute_xyzrgb;

	Base::Property<double> radius;

	void computeNormalsXYZ();
	void computeNormalsXYZRGB();

	/// Input data stream containing point cloud from a given view.
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud_xyz;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;

//	/// Output data stream containing object model point cloud.
//	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> out_cloud_xyz_normals;
//	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb_normals;
	Base::DataStreamOut<pcl::PointCloud<pcl::Normal>::Ptr> out_normals;
};

} //: namespace CuboidNormals
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CuboidNormals", Processors::CuboidNormals::CuboidNormals)

#endif /* CUBOIDNORMALS_HPP_ */
