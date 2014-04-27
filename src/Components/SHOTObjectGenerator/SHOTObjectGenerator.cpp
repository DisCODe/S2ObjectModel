/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SHOTObjectGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SHOTObjectGenerator {

SHOTObjectGenerator::SHOTObjectGenerator(const std::string & name) :
		Base::Component(name)  {

}

SHOTObjectGenerator::~SHOTObjectGenerator() {
}

void SHOTObjectGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool SHOTObjectGenerator::onInit() {

	return true;
}

bool SHOTObjectGenerator::onFinish() {
	return true;
}

bool SHOTObjectGenerator::onStop() {
	return true;
}

bool SHOTObjectGenerator::onStart() {
	return true;
}



} //: namespace SHOTObjectGenerator
} //: namespace Processors
