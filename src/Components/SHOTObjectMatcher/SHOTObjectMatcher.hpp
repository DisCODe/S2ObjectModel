/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SHOTOBJECTMATCHER_HPP_
#define SHOTOBJECTMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace SHOTObjectMatcher {

/*!
 * \class SHOTObjectMatcher
 * \brief SHOTObjectMatcher processor class.
 *
 * SHOTObjectMatcher processor.
 */
class SHOTObjectMatcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SHOTObjectMatcher(const std::string & name = "SHOTObjectMatcher");

	/*!
	 * Destructor
	 */
	virtual ~SHOTObjectMatcher();

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

} //: namespace SHOTObjectMatcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SHOTObjectMatcher", Processors::SHOTObjectMatcher::SHOTObjectMatcher)

#endif /* SHOTOBJECTMATCHER_HPP_ */
