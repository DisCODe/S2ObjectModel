/*!
 * \file
 * \brief 
 * \author Michal Laszkowski
 */

#ifndef S2OMJSONREADER_HPP_
#define S2OMJSONREADER_HPP_

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
namespace S2OMJSONReader {

/*!
 * \class S2OMJSONReader
 * \brief S2OMJSONReader processor class.
 *
 * 
 */
class S2OMJSONReader: public Base::Component, S2ObjectModelFactory {
public:
	/*!
	 * Constructor.
	 */
	S2OMJSONReader(const std::string & name = "S2OMJSONReader");

	/*!
	 * Destructor
	 */
	virtual ~S2OMJSONReader();

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
	Base::DataStreamOut<std::vector<AbstractObject*> > out_models;

	// Handlers
	Base::EventHandler2 h_loadModels;

	// Properties
	Base::Property<string> filenames;

	
	// Handlers
	void loadModels();

};

} //: namespace S2OMJSONReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2OMJSONReader", Processors::S2OMJSONReader::S2OMJSONReader)

#endif /* S2OMJSONREADER_HPP_ */
