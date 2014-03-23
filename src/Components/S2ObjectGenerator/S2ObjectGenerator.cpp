/*!
 * \file
 * \brief
 * \author jkrasnod
 */

#include <memory>
#include <string>

#include "S2ObjectGenerator.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace S2ObjectGenerator {

S2ObjectGenerator::S2ObjectGenerator(const std::string & name) :
		Base::Component(name)  {

}

S2ObjectGenerator::~S2ObjectGenerator() {
}

void S2ObjectGenerator::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool S2ObjectGenerator::onInit() {

	return true;
}

bool S2ObjectGenerator::onFinish() {
	return true;
}

bool S2ObjectGenerator::onStop() {
	return true;
}

bool S2ObjectGenerator::onStart() {
	return true;
}



} //: namespace S2ObjectGenerator
} //: namespace Processors
