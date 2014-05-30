/*!
 * \file
 * \brief 
 * \author jkrasnod
 */

#ifndef SHOT_CONVERTER_HPP_
#define SHOT_CONVERTER_HPP_

/*
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"
*/

#include <Component_Aux.hpp>
#include <Component.hpp>
#include <DataStream.hpp>
#include <Property.hpp>
#include <EventHandler2.hpp>

#include <Types/PointXYZSHOT.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace Processors {
namespace SHOTConverter {

typedef pcl::SHOT352 SHOT;
typedef pcl::PointCloud<SHOT> SHOTCloud;
typedef SHOTCloud::Ptr SHOTCloudPtr;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

typedef pcl::PointCloud<PointXYZSHOT> XYZSHOTCloud;
typedef XYZSHOTCloud::Ptr XYZSHOTCloudPtr;


/*!
 * \class SHOTConverter
 * \brief SHOTConverter processor class.
 *
 * SHOTConverter processor.
 */
class SHOTConverter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOTConverter(const std::string & name = "SHOTConverter");

	/*!
	 * Destructor
	 */
	virtual ~SHOTConverter();

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


// Input data streams

		Base::DataStreamIn<PointCloudPtr> in_points;
		Base::DataStreamIn<PointCloudPtr> in_keypoints;

// Output data streams

		Base::DataStreamOut<SHOTCloudPtr> out_shots;
		Base::DataStreamOut<XYZSHOTCloudPtr> out_cloud_xyzshot;
		

	// Handlers
	Base::EventHandler2 h_process;

	// properties
	Base::Property<float> normal_radius;
	Base::Property<float> shot_radius;

	// Handlers
	void process();


	SHOTCloudPtr getSHOT(PointCloudPtr cloud, NormalCloudPtr normals, PointCloudPtr keypoints);
	NormalCloudPtr getNormals(PointCloudPtr cloud);

};

} //: namespace SHOTConverter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOTConverter", Processors::SHOTConverter::SHOTConverter)

#endif /* SHOT_CONVERTER_HPP_ */
