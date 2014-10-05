/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef REPROJECTIONERROR_HPP_
#define REPROJECTIONERROR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>


namespace Processors {
namespace ReprojectionError {

/*!
 * \class ReprojectionError
 * \brief ReprojectionError processor class.
 *
 * ReprojectionError processor.
 */
class ReprojectionError: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ReprojectionError(const std::string & name = "ReprojectionError");

	/*!
	 * Destructor
	 */
	virtual ~ReprojectionError();

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

	void compute();

	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondences;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_xyz_cloud_expected;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_xyz_cloud_obtained;

	Base::DataStreamOut<double> out_error;
	Base::DataStreamOut<long> out_base;
	Base::DataStreamOut<int> out_group;

	Base::Property<int> group_id;
};

} //: namespace ReprojectionError
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ReprojectionError", Processors::ReprojectionError::ReprojectionError)

#endif /* REPROJECTIONERROR_HPP_ */
