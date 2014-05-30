/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SHOTObjectModelJSONWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace SHOTObjectModelJSONWriter {

SHOTObjectModelJSONWriter::SHOTObjectModelJSONWriter(const std::string & name) :
		Base::Component(name)  {

}

SHOTObjectModelJSONWriter::~SHOTObjectModelJSONWriter() {
}

void SHOTObjectModelJSONWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!

}

bool SHOTObjectModelJSONWriter::onInit() {

	return true;
}

bool SHOTObjectModelJSONWriter::onFinish() {
	return true;
}

bool SHOTObjectModelJSONWriter::onStop() {
	return true;
}

bool SHOTObjectModelJSONWriter::onStart() {
	return true;
}



} //: namespace SHOTObjectModelJSONWriter
} //: namespace Processors
