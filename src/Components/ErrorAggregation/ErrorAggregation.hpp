/*!
 * \file
 * \brief 
 * \author Joanna Krasnodebska
 */

#ifndef ERRORAGGREGATION_HPP_
#define ERRORAGGREGATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace ErrorAggregation {

/*!
 * \class ErrorAggregation
 * \brief ErrorAggregation processor class.
 *
 * ErrorAggregation processor.
 */
class ErrorAggregation: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ErrorAggregation(const std::string & name = "ErrorAggregation");

	/*!
	 * Destructor
	 */
	virtual ~ErrorAggregation();

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

} //: namespace ErrorAggregation
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ErrorAggregation", Processors::ErrorAggregation::ErrorAggregation)

#endif /* ERRORAGGREGATION_HPP_ */
