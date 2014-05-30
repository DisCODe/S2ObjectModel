/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "S2OMJSONWriter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace S2OMJSONWriter {

S2OMJSONWriter::S2OMJSONWriter(const std::string & name) :
		Base::Component(name) , 
		S2OMname("S2OMname", std::string("./") ), 
		dir("dir", std::string("S2OM") ) {
	registerProperty(S2OMname);
	registerProperty(dir);

}

S2OMJSONWriter::~S2OMJSONWriter() {
}

void S2OMJSONWriter::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_s2om", &in_s2om);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("in_cloud_xyzsift", &in_cloud_xyzsift);
	registerStream("in_cloud_xyzshot", &in_cloud_xyzshot);
	registerStream("in_mean_viewpoint_features_number", &in_mean_viewpoint_features_number);
	// Register handlers
	h_Write.setup(boost::bind(&S2OMJSONWriter::Write, this));
	registerHandler("Write", &h_Write);
	addDependency("Write", NULL);

}

bool S2OMJSONWriter::onInit() {

	return true;
}

bool S2OMJSONWriter::onFinish() {
	return true;
}

bool S2OMJSONWriter::onStop() {
	return true;
}

bool S2OMJSONWriter::onStart() {
	return true;
}

void S2OMJSONWriter::Write() {
	LOG(LTRACE) << "S2OMJSONWriter::Write";
	// Try to save the model retrieved from the S2OM data stream.
	if (!in_s2om.empty()) {
		LOG(LDEBUG) << "!in_s2om.empty()";

		// Get S2OM.
		S2ObjectModel* s2om = in_s2om.read();

		// Save point cloud.
		std::string name_cloud_xyzrgb = std::string(dir) + std::string("/") + std::string(S2OMname) + std::string("_xyzrgb.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzrgb, *(s2om->cloud_xyzrgb));
		CLOG(LTRACE) << "Write: saved " << s2om->cloud_xyzrgb->points.size () << " cloud points to "<< name_cloud_xyzrgb;

		// Save SIFT cloud.
		std::string name_cloud_xyzsift = std::string(dir) + std::string("/") + std::string(S2OMname) + std::string("_xyzsift.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzsift, *(s2om->cloud_xyzsift));
		CLOG(LTRACE) << "Write: saved " << s2om->cloud_xyzsift->points.size () << " SIFT points to "<< name_cloud_xyzsift;

		// Save SHOT cloud.
		std::string name_cloud_xyzshot = std::string(dir) + std::string("/") + std::string(S2OMname) + std::string("_xyzshot.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzshot, *(s2om->cloud_xyzshot));
		CLOG(LTRACE) << "Write: saved " << s2om->cloud_xyzshot->points.size () << " SHOT points to "<< name_cloud_xyzshot;
		
		// Save JSON model description.
		ptree ptree_file;
		ptree_file.put("name", S2OMname);
		ptree_file.put("type", "S2ObjectModel");
		ptree_file.put("mean_viewpoint_features_number", s2om->mean_viewpoint_features_number);
		ptree_file.put("cloud_xyzrgb", name_cloud_xyzrgb);
		ptree_file.put("cloud_xyzsift", name_cloud_xyzsift);
		ptree_file.put("cloud_xyzshot", name_cloud_xyzshot);
		write_json (std::string(dir) + std::string("/") + std::string(S2OMname) + std::string(".json"), ptree_file);
		return;
	}

	// Try to save the model retrieved from the three separate data streams.
	if (!in_cloud_xyzrgb.empty() && !in_cloud_xyzsift.empty() && !in_cloud_xyzshot.empty() && !in_mean_viewpoint_features_number.empty()) {
		LOG(LDEBUG) << "!in_cloud_xyzrgb.empty() && !in_cloud_xyzsift.empty() && !in_cloud_xyzshot.empty() && !in_mean_viewpoint_features_number.empty()";

		// Get model from datastreams.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = in_cloud_xyzrgb.read();
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = in_cloud_xyzsift.read();
		pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = in_cloud_xyzshot.read();
		int mean_viewpoint_features_number = in_mean_viewpoint_features_number.read();

		// Save point cloud.
		std::string name_cloud_xyzrgb = std::string(dir) + std::string("/") + std::string(S2OMname) + std::string("_xyzrgb.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzrgb, *(cloud_xyzrgb));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzrgb->points.size () << " cloud points to "<< name_cloud_xyzrgb;

		// Save SIFT cloud.
		std::string name_cloud_xyzsift = std::string(dir) + std::string("/") + std::string(S2OMname) + std::string("_xyzsift.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzsift, *(cloud_xyzsift));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzsift->points.size () << " SIFT points to "<< name_cloud_xyzsift;

		// Save SHOT cloud.
		std::string name_cloud_xyzshot = std::string(dir) + std::string("/") + std::string(S2OMname) + std::string("_xyzshot.pcd");
		pcl::io::savePCDFileASCII (name_cloud_xyzshot, *(cloud_xyzshot));
		CLOG(LTRACE) << "Write: saved " << cloud_xyzshot->points.size () << " SHOT points to "<< name_cloud_xyzshot;
		
		// Save JSON model description.
		ptree ptree_file;
		ptree_file.put("name", S2OMname);
		ptree_file.put("type", "S2ObjectModel");
		ptree_file.put("mean_viewpoint_features_number", mean_viewpoint_features_number);
		ptree_file.put("cloud_xyzrgb", name_cloud_xyzrgb);
		ptree_file.put("cloud_xyzsift", name_cloud_xyzsift);
		ptree_file.put("cloud_xyzshot", name_cloud_xyzshot);
		write_json (std::string(dir) + std::string("/") + std::string(S2OMname) + std::string(".json"), ptree_file);
		return;
	}
	
	CLOG(LWARNING) << "There are no required datastreams enabling save of the SOM to file.";	
}



} //: namespace S2OMJSONWriter
} //: namespace Processors
