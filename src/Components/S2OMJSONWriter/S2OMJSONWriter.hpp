/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef S2OMJSONWRITER_HPP_
#define S2OMJSONWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/S2ObjectModel.hpp> 

namespace Processors {
namespace S2OMJSONWriter {

/*!
 * \class S2OMJSONWriter
 * \brief S2OMJSONWriter processor class.
 *
 * 
 */
class S2OMJSONWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	S2OMJSONWriter(const std::string & name = "S2OMJSONWriter");

	/*!
	 * Destructor
	 */
	virtual ~S2OMJSONWriter();

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
	Base::DataStreamIn<S2ObjectModel*> in_s2om;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_cloud_xyzrgb;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr> in_cloud_xyzsift;
	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr> in_cloud_xyzshot;
	Base::DataStreamIn<int> in_mean_viewpoint_features_number;

	// Output data streams

	// Handlers
	Base::EventHandler2 h_Write;
	Base::EventHandler2 h_on_cloud_xyzrgb;
	Base::EventHandler2 h_on_cloud_xyzsift;
	Base::EventHandler2 h_on_cloud_xyzshot;

	// Properties
	Base::Property<std::string> S2OMname;
	Base::Property<std::string> dir;

	
	// Handlers
	void Write();
	void on_cloud_xyzrgb();
	void on_cloud_xyzsift();
	void on_cloud_xyzshot();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb; 
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift; 
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot; 

};

} //: namespace S2OMJSONWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2OMJSONWriter", Processors::S2OMJSONWriter::S2OMJSONWriter)

#endif /* S2OMJSONWRITER_HPP_ */
