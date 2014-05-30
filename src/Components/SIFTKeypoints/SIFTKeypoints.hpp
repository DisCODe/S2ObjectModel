/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef SIFTKEYPOINTS_HPP_
#define SIFTKEYPOINTS_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace SIFTKeypoints {

/*!
 * \class SIFTKeypoints
 * \brief SIFTKeypoints processor class.
 *
 * SIFTKeypoints processor.
 */
class SIFTKeypoints: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	SIFTKeypoints(const std::string & name = "SIFTKeypoints");

	/*!
	 * Destructor
	 */
	virtual ~SIFTKeypoints();

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

} //: namespace SIFTKeypoints
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("SIFTKeypoints", Processors::SIFTKeypoints::SIFTKeypoints)

#endif /* SIFTKEYPOINTS_HPP_ */
