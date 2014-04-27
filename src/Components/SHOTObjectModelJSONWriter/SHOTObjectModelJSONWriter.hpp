/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SHOTOBJECTMODELJSONWRITER_HPP_
#define SHOTOBJECTMODELJSONWRITER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace SHOTObjectModelJSONWriter {

/*!
 * \class SHOTObjectModelJSONWriter
 * \brief SHOTObjectModelJSONWriter processor class.
 *
 * SHOTObjectModelJSONWriter processor.
 */
class SHOTObjectModelJSONWriter: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOTObjectModelJSONWriter(const std::string & name = "SHOTObjectModelJSONWriter");

	/*!
	 * Destructor
	 */
	virtual ~SHOTObjectModelJSONWriter();

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

} //: namespace SHOTObjectModelJSONWriter
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOTObjectModelJSONWriter", Processors::SHOTObjectModelJSONWriter::SHOTObjectModelJSONWriter)

#endif /* SHOTOBJECTMODELJSONWRITER_HPP_ */
