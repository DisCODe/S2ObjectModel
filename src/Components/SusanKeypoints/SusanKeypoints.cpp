/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SusanKeypoints.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SusanKeypoints {

SusanKeypoints::SusanKeypoints(const std::string & name) :
		Base::Component(name)  {

}

SusanKeypoints::~SusanKeypoints() {
}

void SusanKeypoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool SusanKeypoints::onInit() {

	return true;
}

bool SusanKeypoints::onFinish() {
	return true;
}

bool SusanKeypoints::onStop() {
	return true;
}

bool SusanKeypoints::onStart() {
	return true;
}



} //: namespace SusanKeypoints
} //: namespace Processors
