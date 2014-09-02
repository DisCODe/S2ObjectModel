/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef SINGLES2OMJSONREADER_HPP_
#define SINGLES2OMJSONREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <Types/PointCloudObject.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>

#include <Types/S2ObjectModelFactory.hpp>

namespace Processors {
namespace SingleS2OMJsonReader {

/*!
 * \class SingleS2OMJsonReader
 * \brief SingleS2OMJsonReader processor class.
 *
 *
 */
class SingleS2OMJsonReader: public Base::Component, S2ObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	SingleS2OMJsonReader(const std::string & name = "SingleS2OMJsonReader");

	/*!
	 * Destructor
	 */
	virtual ~SingleS2OMJsonReader();

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

	// Output data streams
	Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_cloud_xyzrgb;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_cloud_xyzsift;
	Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_cloud_xyzshot;
	Base::DataStreamOut<std::string> out_name;

	// Handlers
	Base::EventHandler2 h_loadModel;

	// Properties
	Base::Property<string> filenames;

	
	// Handlers
	void loadModel();

};

} //: namespace SingleS2OMJsonReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SingleS2OMJsonReader", Processors::SingleS2OMJsonReader::SingleS2OMJsonReader)

#endif /* SINGLES2OMJSONREADER_HPP_ */
