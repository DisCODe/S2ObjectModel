/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef S2OBJECTMATCHER_HPP_
#define S2OBJECTMATCHER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>



namespace Processors {
namespace S2ObjectMatcher {

/*!
 * \class S2ObjectMatcher
 * \brief S2ObjectMatcher processor class.
 *
 * S2ObjectMatcher processor.
 */
class S2ObjectMatcher: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	S2ObjectMatcher(const std::string & name = "S2ObjectMatcher");

	/*!
	 * Destructor
	 */
	virtual ~S2ObjectMatcher();

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

} //: namespace S2ObjectMatcher
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("S2ObjectMatcher", Processors::S2ObjectMatcher::S2ObjectMatcher)

#endif /* S2OBJECTMATCHER_HPP_ */
