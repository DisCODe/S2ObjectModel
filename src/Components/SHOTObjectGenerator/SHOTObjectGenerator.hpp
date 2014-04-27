/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SHOTOBJECTGENERATOR_HPP_
#define SHOTOBJECTGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace SHOTObjectGenerator {

/*!
 * \class SHOTObjectGenerator
 * \brief SHOTObjectGenerator processor class.
 *
 * SHOTObjectGenerator processor.
 */
class SHOTObjectGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOTObjectGenerator(const std::string & name = "SHOTObjectGenerator");

	/*!
	 * Destructor
	 */
	virtual ~SHOTObjectGenerator();

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

} //: namespace SHOTObjectGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOTObjectGenerator", Processors::SHOTObjectGenerator::SHOTObjectGenerator)

#endif /* SHOTOBJECTGENERATOR_HPP_ */
