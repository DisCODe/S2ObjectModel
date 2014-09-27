/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef NORMALS_HPP_
#define NORMALS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


namespace Processors {
namespace Normals {

/*!
 * \class Normals
 * \brief Normals processor class.
 *
 * Normals processor.
 */
class Normals: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Normals(const std::string & name = "Normals");

	/*!
	 * Destructor
	 */
	virtual ~Normals();

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
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_cloud_xyzrgb_normals;
	Base::DataStreamOut<pcl::PointCloud<pcl::Normal>::Ptr> out_cloud_normals;

};

} //: namespace Normals
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Normals", Processors::Normals::Normals)

#endif /* NORMALS_HPP_ */
