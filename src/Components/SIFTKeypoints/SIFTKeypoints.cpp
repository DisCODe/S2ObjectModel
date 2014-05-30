/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SIFTKeypoints.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SIFTKeypoints {

SIFTKeypoints::SIFTKeypoints(const std::string & name) :
		Base::Component(name)  {

}

SIFTKeypoints::~SIFTKeypoints() {
}

void SIFTKeypoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool SIFTKeypoints::onInit() {

	return true;
}

bool SIFTKeypoints::onFinish() {
	return true;
}

bool SIFTKeypoints::onStop() {
	return true;
}

bool SIFTKeypoints::onStart() {
	return true;
}



} //: namespace SIFTKeypoints
} //: namespace Processors
