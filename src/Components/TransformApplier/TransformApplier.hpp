/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef TRANSFORMAPPLIER_HPP_
#define TRANSFORMAPPLIER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"



namespace Processors {
namespace TransformApplier {

/*!
 * \class TransformApplier
 * \brief TransformApplier processor class.
 *
 * TransformApplier processor.
 */
class TransformApplier: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	TransformApplier(const std::string & name = "TransformApplier");

	/*!
	 * Destructor
	 */
	virtual ~TransformApplier();

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

} //: namespace TransformApplier
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("TransformApplier", Processors::TransformApplier::TransformApplier)

#endif /* TRANSFORMAPPLIER_HPP_ */
