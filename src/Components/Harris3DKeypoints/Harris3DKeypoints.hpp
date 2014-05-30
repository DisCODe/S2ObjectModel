/*!
 * \file
 * \brief 
 * \author jkrasnod
 */

#ifndef HARRIS3DKEYPOINTS_HPP_
#define HARRIS3DKEYPOINTS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

/*
typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> pcl::PointCloud<pcl::PointXYZ>;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::PointCloud<pcl::PointXYZ>::Ptr;

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> pcl::PointCloud<pcl::PointXYZI>;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcl::PointCloud<pcl::PointXYZI>::Ptr;*/

namespace Processors {
namespace Harris3DKeypoints {

/*!
 * \class Harris3DKeypoints
 * \brief Harris3DKeypoints processor class.
 *
 * Harris3DKeypoints processor.
 */
class Harris3DKeypoints: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	Harris3DKeypoints(const std::string & name = "Harris3DKeypoints");

	/*!
	 * Destructor
	 */
	virtual ~Harris3DKeypoints();

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

	Base::Property<double> radius;
	Base::Property<double> radius_search;


	// Handlers
    void compute();
};

} //: namespace Harris3DKeypoints
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("Harris3DKeypoints", Processors::Harris3DKeypoints::Harris3DKeypoints)

#endif /* HARRIS3DKEYPOINTS_HPP_ */
