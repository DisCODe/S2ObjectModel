/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "TransformApplier.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace TransformApplier {

TransformApplier::TransformApplier(const std::string & name) :
		Base::Component(name)  {

}

TransformApplier::~TransformApplier() {
}

void TransformApplier::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool TransformApplier::onInit() {

	return true;
}

bool TransformApplier::onFinish() {
	return true;
}

bool TransformApplier::onStop() {
	return true;
}

bool TransformApplier::onStart() {
	return true;
}



} //: namespace TransformApplier
} //: namespace Processors
