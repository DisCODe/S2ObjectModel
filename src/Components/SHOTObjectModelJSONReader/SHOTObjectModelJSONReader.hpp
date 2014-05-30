/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SHOTOBJECTMODELJSONREADER_HPP_
#define SHOTOBJECTMODELJSONREADER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace SHOTObjectModelJSONReader {

/*!
 * \class SHOTObjectModelJSONReader
 * \brief SHOTObjectModelJSONReader processor class.
 *
 * SHOTObjectModelJSONReader processor.
 */
class SHOTObjectModelJSONReader: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOTObjectModelJSONReader(const std::string & name = "SHOTObjectModelJSONReader");

	/*!
	 * Destructor
	 */
	virtual ~SHOTObjectModelJSONReader();

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


	

};

} //: namespace SHOTObjectModelJSONReader
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOTObjectModelJSONReader", Processors::SHOTObjectModelJSONReader::SHOTObjectModelJSONReader)

#endif /* SHOTOBJECTMODELJSONREADER_HPP_ */
