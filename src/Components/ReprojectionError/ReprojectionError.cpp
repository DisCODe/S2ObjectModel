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
		Base::Component(name)  {

}

ReprojectionError::~ReprojectionError() {
}

void ReprojectionError::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_correspondences", &in_correspondences);
	registerStream("in_xyz_cloud_expected", &in_xyz_cloud_expected);
	registerStream("in_xyz_cloud_obtained", &in_xyz_cloud_obtained);
	registerStream("out_error", &out_error);

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
	pcl::CorrespondencesPtr correspondences = in_correspondences.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr expected = in_xyz_cloud_expected.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr obtained = in_xyz_cloud_obtained.read();

	double sum = 0.0;

	for (int i; i < correspondences->size(); ++i) {
		pcl::Correspondence corr = correspondences->at(i);
		pcl::PointXYZ expectedPoint = expected->at(corr.index_match);
		pcl::PointXYZ obtainedPoint = obtained->at(corr.index_query);
		sum += std::sqrt(
				std::pow(expectedPoint.x - obtainedPoint.x, 2.0) +
				std::pow(expectedPoint.y - obtainedPoint.y, 2.0) +
				std::pow(expectedPoint.z - obtainedPoint.z, 2.0));

	}

	out_error.write(sum);
}



} //: namespace ReprojectionError
} //: namespace Processors
