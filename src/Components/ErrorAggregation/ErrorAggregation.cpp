/*!
 * \file
 * \brief
 * \author Joanna Krasnodebska
 */

#include <memory>
#include <string>

#include "ErrorAggregation.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ErrorAggregation {

ErrorAggregation::ErrorAggregation(const std::string & name) :
		Base::Component(name)  {

}

ErrorAggregation::~ErrorAggregation() {
}

void ErrorAggregation::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool ErrorAggregation::onInit() {

	return true;
}

bool ErrorAggregation::onFinish() {
	return true;
}

bool ErrorAggregation::onStop() {
	return true;
}

bool ErrorAggregation::onStart() {
	return true;
}



} //: namespace ErrorAggregation
} //: namespace Processors
