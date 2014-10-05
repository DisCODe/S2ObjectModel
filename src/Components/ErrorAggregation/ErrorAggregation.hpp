/*!
 * \file
 * \brief 
 * \author Joanna Krasnodebska
 */

#ifndef ERRORAGGREGATION_HPP_
#define ERRORAGGREGATION_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <vector>



namespace Processors {
namespace ErrorAggregation {

/*!
 * \class ErrorAggregation
 * \brief ErrorAggregation processor class.
 *
 * ErrorAggregation processor.
 */
class ErrorAggregation: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	ErrorAggregation(const std::string & name = "ErrorAggregation");

	/*!
	 * Destructor
	 */
	virtual ~ErrorAggregation();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();

	void addToGroup1();
	void addToGroup2();
	void addToGroup3();
	void addToGroup4();
	void displayResults();

	Base::DataStreamIn<double> in_error_1;
	Base::DataStreamIn<long> in_base_1;


	Base::DataStreamIn<double> in_error_2;
	Base::DataStreamIn<long> in_base_2;


	Base::DataStreamIn<double> in_error_3;
	Base::DataStreamIn<long> in_base_3;


	Base::DataStreamIn<double> in_error_4;
	Base::DataStreamIn<long> in_base_4;

	Base::Property<std::string> group1_name;
	Base::Property<std::string> group2_name;
	Base::Property<std::string> group3_name;
	Base::Property<std::string> group4_name;


private:

	class Result {

	private:
		long base;
		double error;

	public:
		Result(long _base, double _error) {
			base = _base;
			error = _error;
		}

		 friend std::ostream & operator<< (std::ostream &out, const Result &s) {
			 return out << "error : " << s.error << ", base : " << s.base << "\n";
		}

		 long getBase() {
			 return base;
		 }

		 double getError() {
			 return error;
		 }
	};

	class ResultVector {

	public:
		std::vector<Result> results;

		void add(long base, double error) {
			Result result(base, error);
			results.push_back(result);
		}

		friend std::ostream & operator<< (std::ostream &out, const ResultVector &s) {
			out << "Results (" << s.results.size() << "):\n";
			for (int i = 0; i < s.results.size();++i) {
				out << "\t" << s.results[i];
			}
			return out;
		}

		double getErrorSum() {
			double sum = 0;
			for (int i = 0; i < results.size();++i) {
				sum += results[i].getError();
			}
			return sum;
		}

		double getAvgError() {
			double sum = getErrorSum();
			return sum/(double)results.size();
		}
	};

	ResultVector group1;
	ResultVector group2;
	ResultVector group3;
	ResultVector group4;

};

} //: namespace ErrorAggregation
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("ErrorAggregation", Processors::ErrorAggregation::ErrorAggregation)

#endif /* ERRORAGGREGATION_HPP_ */
