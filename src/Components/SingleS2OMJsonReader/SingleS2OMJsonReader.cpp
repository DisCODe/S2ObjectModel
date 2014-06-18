/*!
 * \file
 * \brief
 * \author jkrasnod
 */

#include <memory>
#include <string>

#include "SingleS2OMJsonReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace SingleS2OMJsonReader {

SingleS2OMJsonReader::SingleS2OMJsonReader(const std::string & name) :
		Base::Component(name), filenames("filenames", string("./")) {
	registerProperty(filenames);

}

SingleS2OMJsonReader::~SingleS2OMJsonReader() {
}

void SingleS2OMJsonReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);
	// Register handlers
	h_loadModel.setup(boost::bind(&SingleS2OMJsonReader::loadModel, this));
	registerHandler("loadModel", &h_loadModel);
	addDependency("loadModel", NULL);

}

bool SingleS2OMJsonReader::onInit() {

	return true;
}

bool SingleS2OMJsonReader::onFinish() {
	return true;
}

bool SingleS2OMJsonReader::onStop() {
	return true;
}

bool SingleS2OMJsonReader::onStart() {
	return true;
}

void SingleS2OMJsonReader::loadModel() {
	LOG(LTRACE) << "SingleS2OMJsonReader::loadModel()";

	std::string s = filenames;

	// Temporary variables - names.
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;
	std::string name_cloud_xyzshot;

	// Iterate through JSON files.

	ptree ptree_file;
	try {
		// Open JSON file and load it to ptree.
		read_json(s, ptree_file);
		// Read JSON properties.
		model_name = ptree_file.get < std::string > ("name");
		mean_viewpoint_features_number = ptree_file.get<int>(
				"mean_viewpoint_features_number");
		name_cloud_xyzrgb = ptree_file.get < std::string > ("cloud_xyzrgb");
		name_cloud_xyzsift = ptree_file.get < std::string > ("cloud_xyzsift");
		name_cloud_xyzshot = ptree_file.get < std::string > ("cloud_xyzshot");
	} //: try
	catch (std::exception const& e) {
		LOG(LERROR) << "SingleS2OMJsonReader: file " << s
				<< " not found or invalid\n";
		return;
	} //: catch

	LOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
	LOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
	LOG(LDEBUG) << "name_cloud_xyzshot:" << name_cloud_xyzshot;

	// Read XYZRGB cloud.
	cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_cloud_xyzrgb, *cloud_xyzrgb)
			== -1) {
		LOG(LERROR) << "SingleS2OMJsonReader: file " << name_cloud_xyzrgb
				<< " not found\n";
		return;
	} //: if

	// Read SIFT cloud.
	cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr(
			new pcl::PointCloud<PointXYZSIFT>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<PointXYZSIFT>(name_cloud_xyzsift, *cloud_xyzsift)
			== -1) {
		LOG(LERROR) << "SingleS2OMJsonReader: file " << name_cloud_xyzsift
				<< " not found\n";
		return;
	} //: if

	// Read SHOT cloud.
	cloud_xyzshot = pcl::PointCloud<PointXYZSHOT>::Ptr(
			new pcl::PointCloud<PointXYZSHOT>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<PointXYZSHOT>(name_cloud_xyzshot, *cloud_xyzshot)
			== -1) {
		LOG(LERROR) << "SingleS2OMJsonReader: file " << name_cloud_xyzshot
				<< " not found\n";
		return;
	} //: if

	out_cloud_xyzrgb.write(cloud_xyzrgb);
	out_cloud_xyzsift.write(cloud_xyzsift);
	out_cloud_xyzshot.write(cloud_xyzshot);


}

} //: namespace SingleS2OMJsonReader
} //: namespace Processors
