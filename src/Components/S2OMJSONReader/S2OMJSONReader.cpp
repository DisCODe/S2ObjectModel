/*!
 * \file
 * \brief
 * \author Michal Laszkowski
 */

#include <memory>
#include <string>

#include "S2OMJSONReader.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace S2OMJSONReader {

S2OMJSONReader::S2OMJSONReader(const std::string & name) :
		Base::Component(name) , 
		filenames("filenames", string("./")) {
	registerProperty(filenames);

}

S2OMJSONReader::~S2OMJSONReader() {
}

void S2OMJSONReader::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("out_models", &out_models);
	// Register handlers
	h_loadModels.setup(boost::bind(&S2OMJSONReader::loadModels, this));
	registerHandler("loadModels", &h_loadModels);
//	addDependency("loadModels", NULL);
}

bool S2OMJSONReader::onInit() {

	return true;
}

bool S2OMJSONReader::onFinish() {
	return true;
}

bool S2OMJSONReader::onStop() {
	return true;
}

bool S2OMJSONReader::onStart() {
	return true;
}

void S2OMJSONReader::loadModels() {
	CLOG(LWARNING) << "S2OMJSONReader::loadModels";

	// List of the returned S2OMs.
	std::vector<AbstractObject*> models;
	
	// Names of models/JSON files.	
	std::vector<std::string> namesList;
	std::string s= filenames;
	boost::split(namesList, s, boost::is_any_of(";"));

	// Temporary variables - names.
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;
	std::string name_cloud_xyzshot;
	
	// Iterate through JSON files.
	for (size_t i = 0; i < namesList.size(); i++){
		ptree ptree_file;
		try{
			// Open JSON file and load it to ptree.
			read_json(namesList[i], ptree_file);
			// Read JSON properties.
			model_name = ptree_file.get<std::string>("name");
			mean_viewpoint_features_number = ptree_file.get<int>("mean_viewpoint_features_number");
			name_cloud_xyzrgb = ptree_file.get<std::string>("cloud_xyzrgb");
			name_cloud_xyzsift = ptree_file.get<std::string>("cloud_xyzsift");
			name_cloud_xyzshot = ptree_file.get<std::string>("cloud_xyzshot");
		}//: try
		catch(std::exception const& e){
			LOG(LERROR) << "S2OMJSONReader: file "<< namesList[i] <<" not found or invalid\n";
			continue;	
		}//: catch

		CLOG(LDEBUG) << "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
		CLOG(LDEBUG) << "name_cloud_xyzsift:" << name_cloud_xyzsift;
		CLOG(LDEBUG) << "name_cloud_xyzshot:" << name_cloud_xyzshot;
		

		// Read XYZRGB cloud.
		cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (name_cloud_xyzrgb, *cloud_xyzrgb) == -1) 
		{
			CLOG(LERROR) << "S2OMJSONReader: file "<< name_cloud_xyzrgb <<" not found\n";
			continue;
		}//: if

		// Read SIFT cloud.
		cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr (new pcl::PointCloud<PointXYZSIFT>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<PointXYZSIFT> (name_cloud_xyzsift, *cloud_xyzsift) == -1) 
		{
			CLOG(LERROR) << "S2OMJSONReader: file "<< name_cloud_xyzsift <<" not found\n";
			continue;
		}//: if
		
		// Read SHOT cloud.
		cloud_xyzshot = pcl::PointCloud<PointXYZSHOT>::Ptr (new pcl::PointCloud<PointXYZSHOT>());
		// Try to load the file.
		if (pcl::io::loadPCDFile<PointXYZSHOT> (name_cloud_xyzshot, *cloud_xyzshot) == -1) 
		{
			CLOG(LERROR) << "S2OMJSONReader: file "<< name_cloud_xyzshot <<" not found\n";
			continue;
		}//: if

		// Create S2OModel and add it to list.
		S2ObjectModel* model;
		model = dynamic_cast<S2ObjectModel*>(produce());
		models.push_back(model);

	}//: for

	// Push models to output datastream.
	CLOG(LWARNING) << "S2OMJSONReader::out_models.write";
	out_models.write(models);
}



} //: namespace S2OMJSONReader
} //: namespace Processors
