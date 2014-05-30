/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SHOTObjectMatcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SHOTObjectMatcher {

SHOTObjectMatcher::SHOTObjectMatcher(const std::string & name) :
		Base::Component(name)  {

}

SHOTObjectMatcher::~SHOTObjectMatcher() {
}

void SHOTObjectMatcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool SHOTObjectMatcher::onInit() {

	return true;
}

bool SHOTObjectMatcher::onFinish() {
	return true;
}

bool SHOTObjectMatcher::onStop() {
	return true;
}

bool SHOTObjectMatcher::onStart() {
	return true;
}



} //: namespace SHOTObjectMatcher
} //: namespace Processors
