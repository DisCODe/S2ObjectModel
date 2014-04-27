/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef ISS3DKEYPOINTS_HPP_
#define ISS3DKEYPOINTS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/pcl_base.h>
#include <boost/function.hpp>

namespace Processors {
namespace ISS3DKeypoints {

/*!
 * \class ISS3DKeypoints
 * \brief ISS3DKeypoints processor class.
 *
 * ISS3DKeypoints processor.
 */
class ISS3DKeypoints: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ISS3DKeypoints(const std::string & name = "ISS3DKeypoints");

	/*!
	 * Destructor
	 */
	virtual ~ISS3DKeypoints();

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
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_cloud;

	/// Output data stream containing object model point cloud.
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_keypoints;

	// Handlers
    Base::EventHandler2 h_compute;

	// Handlers
    void compute();
	

};

} //: namespace ISS3DKeypoints
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ISS3DKeypoints", Processors::ISS3DKeypoints::ISS3DKeypoints)

#endif /* ISS3DKEYPOINTS_HPP_ */
