/*!
 * \file
 * \brief 
 * \author jkrasnod
 */

#ifndef S2OBJECTGENERATOR_HPP_
#define S2OBJECTGENERATOR_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace S2ObjectGenerator {

/*!
 * \class S2ObjectGenerator
 * \brief S2ObjectGenerator processor class.
 *
 * S2ObjectGenerator processor.
 */
class S2ObjectGenerator: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	S2ObjectGenerator(const std::string & name = "S2ObjectGenerator");

	/*!
	 * Destructor
	 */
	virtual ~S2ObjectGenerator();

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

} //: namespace S2ObjectGenerator
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2ObjectGenerator", Processors::S2ObjectGenerator::S2ObjectGenerator)

#endif /* S2OBJECTGENERATOR_HPP_ */
