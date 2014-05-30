/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SHOTObjectModelJSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SHOTObjectModelJSONReader {

SHOTObjectModelJSONReader::SHOTObjectModelJSONReader(const std::string & name) :
		Base::Component(name)  {

}

SHOTObjectModelJSONReader::~SHOTObjectModelJSONReader() {
}

void SHOTObjectModelJSONReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool SHOTObjectModelJSONReader::onInit() {

	return true;
}

bool SHOTObjectModelJSONReader::onFinish() {
	return true;
}

bool SHOTObjectModelJSONReader::onStop() {
	return true;
}

bool SHOTObjectModelJSONReader::onStart() {
	return true;
}



} //: namespace SHOTObjectModelJSONReader
} //: namespace Processors
