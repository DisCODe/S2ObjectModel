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

	registerStream("in_error_1", &in_error_1);
	registerStream("in_base_1", &in_base_1);

	registerStream("in_error_2", &in_error_2);
	registerStream("in_base_2", &in_base_2);

	registerStream("in_error_3", &in_error_3);
	registerStream("in_base_3", &in_base_3);

	registerStream("in_error_4", &in_error_4);
	registerStream("in_base_4", &in_base_4);

	registerHandler("group1", boost::bind(&ErrorAggregation::addToGroup1, this));
	addDependency("group1", &in_error_1);
	addDependency("group1", &in_base_1);

	registerHandler("group2", boost::bind(&ErrorAggregation::addToGroup2, this));
	addDependency("group2", &in_error_2);
	addDependency("group2", &in_base_2);

	registerHandler("group3", boost::bind(&ErrorAggregation::addToGroup3, this));
	addDependency("group3", &in_error_3);
	addDependency("group3", &in_base_3);

	registerHandler("group4", boost::bind(&ErrorAggregation::addToGroup4, this));
	addDependency("group4", &in_error_4);
	addDependency("group4", &in_base_4);

	registerHandler("displayResults", boost::bind(&ErrorAggregation::displayResults, this));
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

void ErrorAggregation::addToGroup1() {
	CLOG(LWARNING) << "ErrorAggregation::addToGroup1";

	double error = in_error_1.read();
	long base = in_base_1.read();

	CLOG(LWARNING) << "ErrorAggregation::(error, base) =(" << error << ", " << base << ")";

	group1.add(base, error);
}

void ErrorAggregation::addToGroup2() {
	CLOG(LWARNING) << "ErrorAggregation::addToGroup2";

	double error = in_error_2.read();
	long base = in_base_2.read();

	CLOG(LWARNING) << "ErrorAggregation::(error, base) =(" << error << ", " << base << ")";

	group2.add(base, error);
}

void ErrorAggregation::addToGroup3() {
	CLOG(LWARNING) << "ErrorAggregation::addToGroup3";

	double error = in_error_3.read();
	long base = in_base_3.read();

	CLOG(LWARNING) << "ErrorAggregation::(error, base) =(" << error << ", " << base << ")";

	group3.add(base, error);
}

void ErrorAggregation::addToGroup4() {
	CLOG(LWARNING) << "ErrorAggregation::addToGroup4";

	double error = in_error_4.read();
	long base = in_base_4.read();

	CLOG(LWARNING) << "ErrorAggregation::(error, base) =(" << error << ", " << base << ")";

	group4.add(base, error);
}

void ErrorAggregation::displayResults() {
	CLOG(LWARNING) << "ErrorAggregation::displayResults";
	CLOG(LWARNING) << "Results:\ngroup1 " << group1 << "\ngroup2 " << group2
			<< "\ngroup3 " << group3
			<< "\ngroup4 " << group4;
}



} //: namespace ErrorAggregation
} //: namespace Processors
