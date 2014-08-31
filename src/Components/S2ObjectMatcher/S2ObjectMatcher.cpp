/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "S2ObjectMatcher.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace S2ObjectMatcher {

S2ObjectMatcher::S2ObjectMatcher(const std::string & name) :
		Base::Component(name)  {

}

S2ObjectMatcher::~S2ObjectMatcher() {
}

void S2ObjectMatcher::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool S2ObjectMatcher::onInit() {

	return true;
}

bool S2ObjectMatcher::onFinish() {
	return true;
}

bool S2ObjectMatcher::onStop() {
	return true;
}

bool S2ObjectMatcher::onStart() {
	return true;
}



} //: namespace S2ObjectMatcher
} //: namespace Processors
