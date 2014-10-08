/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>
#include <math.h>

#include "ReprojectionError.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace ReprojectionError {

ReprojectionError::ReprojectionError(const std::string & name) :
		Base::Component(name), group_id("group_id", 1) {
	registerProperty(group_id);
}

ReprojectionError::~ReprojectionError() {
}

void ReprojectionError::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_correspondences", &in_correspondences);
	registerStream("in_xyz_cloud_expected", &in_xyz_cloud_expected);
	registerStream("in_xyz_cloud_obtained", &in_xyz_cloud_obtained);
	registerStream("out_error", &out_error);
	registerStream("out_base", &out_base);
	registerStream("out_group", &out_group);

    registerHandler("compute", boost::bind(&ReprojectionError::compute, this));
    addDependency("compute", &in_correspondences);
    addDependency("compute", &in_xyz_cloud_expected);
    addDependency("compute", &in_xyz_cloud_obtained);
}

bool ReprojectionError::onInit() {

	return true;
}

bool ReprojectionError::onFinish() {
	return true;
}

bool ReprojectionError::onStop() {
	return true;
}

bool ReprojectionError::onStart() {
	return true;
}

void ReprojectionError::compute() {
	CLOG(LWARNING) << "ReprojectionError::compute(" << group_id << ")";
	pcl::CorrespondencesPtr correspondences = in_correspondences.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr expected = in_xyz_cloud_expected.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr obtained = in_xyz_cloud_obtained.read();

	CLOG(LWARNING) << "ReprojectionError::in_correspondences.size()" << correspondences->size();
	CLOG(LWARNING) << "ReprojectionError:: expected size :" << expected->size() <<", obtained: "<< obtained->size();

	double sum = 0.0;

	for (int i = 0; i < correspondences->size(); ++i) {
		pcl::Correspondence corr = correspondences->at(i);
		if (corr.index_match >= expected->size()) {
			CLOG(LERROR) << "Index match out of bound (expected point)";
		}
		if (corr.index_query >= obtained->size()) {
			CLOG(LERROR) << "Index query out of bound (expected point)";
		}
		pcl::PointXYZ expectedPoint = expected->at(corr.index_match);
		pcl::PointXYZ obtainedPoint = obtained->at(corr.index_query);
		sum += std::sqrt(
				std::pow((double)expectedPoint.x - (double)obtainedPoint.x, 2.0) +
				std::pow((double)expectedPoint.y - (double)obtainedPoint.y, 2.0) +
				std::pow((double)expectedPoint.z - (double)obtainedPoint.z, 2.0)
		);
	}

	CLOG(LWARNING) << "ReprojectionError::out_error.write (" << group_id << "): " << sum << "/" << correspondences->size();

	out_error.write(sum);
	out_base.write(correspondences->size());
	out_group.write(group_id);
}



} //: namespace ReprojectionError
} //: namespace Processors
