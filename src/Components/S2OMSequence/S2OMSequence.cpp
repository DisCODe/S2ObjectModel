/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "S2OMSequence.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "Utils.hpp"

#include <algorithm>
#include <boost/foreach.hpp>

#include <opencv2/highgui/highgui.hpp>

using boost::property_tree::ptree;
using boost::property_tree::read_json;
using boost::property_tree::write_json;

namespace Processors {
namespace S2OMSequence {

S2OMSequence::S2OMSequence(const std::string & name) :
		Base::Component(name), prop_directory("sequence.directory", std::string(".")), prop_sort("mode.sort", true), prop_loop(
				"mode.loop", false), prop_auto_trigger("mode.auto_trigger", true), read_on_init("read_on_init", true) {
	registerProperty(prop_directory);
	registerProperty(prop_sort);
	registerProperty(prop_loop);
	registerProperty(prop_auto_trigger);
	registerProperty(read_on_init);

	// Set first frame index number.
	if (prop_auto_trigger)
		frame = -1;
	else
		frame = 0;

	// Initialize flags
	next_image_flag = false;
	reload_flag = false;

}

S2OMSequence::~S2OMSequence() {
}

void S2OMSequence::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	// Register streams.
	registerStream("in_trigger", &in_trigger);
	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);
	registerStream("out_cloud_xyzsift", &out_cloud_xyzsift);
	registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);
	registerStream("out_name", &out_name);

	// Register handlers - loads image, NULL dependency.
	registerHandler("onLoadImage", boost::bind(&S2OMSequence::onLoadImage, this));
	addDependency("onLoadImage", NULL);

	// Register handlers - next image, can be triggered manually (from GUI) or by new data present in_load_next_image_trigger dataport.
	// 1st version - manually.
	registerHandler("Next image", boost::bind(&S2OMSequence::onLoadNextImage, this));

	// 2nd version - external trigger.
	registerHandler("onTriggeredLoadNextImage", boost::bind(&S2OMSequence::onTriggeredLoadNextImage, this));
	addDependency("onTriggeredLoadNextImage", &in_trigger);

	// Register handlers - reloads sequence, triggered manually.
	registerHandler("Reload sequence", boost::bind(&S2OMSequence::onSequenceReload, this));
}

bool S2OMSequence::onInit() {
	// Load files on first
	reload_flag = true;
	if (read_on_init)
		next_image_flag = true;
	return true;
}

bool S2OMSequence::onFinish() {
	return true;
}

void S2OMSequence::readModel(std::string filename) {
	std::string name_cloud_xyzrgb;
	std::string name_cloud_xyzsift;
	std::string name_cloud_xyzshot;
	std::string model_name;
	std::string type;
	int mean_viewpoint_features_number;

	std::string s = filename;

	// Iterate through JSON files.

	ptree ptree_file;
	try {

		// Open JSON file and load it to ptree.
		read_json(s, ptree_file);
		// Read JSON properties.
		model_name = ptree_file.get<std::string>("name");
		name_cloud_xyzrgb = ptree_file.get<std::string>("cloud_xyzrgb");
		name_cloud_xyzsift = ptree_file.get<std::string>("cloud_xyzsift");
		name_cloud_xyzshot = ptree_file.get<std::string>("cloud_xyzshot");
	//	type = ptree_file.get<std::string>("type");
	//	mean_viewpoint_features_number = ptree_file.get<int>("mean_viewpoint_features_number");
	} //: try
	catch (std::exception const& e) {
		LOG(LERROR)<< "S2OMSequence: file " << s
		<< " not found or invalid\n" << e.what();
		return;
	} //: catch

	LOG(LDEBUG)<< "name_cloud_xyzrgb:" << name_cloud_xyzrgb;
	LOG(LDEBUG)<< "name_cloud_xyzsift:" << name_cloud_xyzsift;
	LOG(LDEBUG)<< "name_cloud_xyzshot:" << name_cloud_xyzshot;

	// Read XYZRGB cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
			new pcl::PointCloud<pcl::PointXYZRGB>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(name_cloud_xyzrgb, *cloud_xyzrgb) == -1) {
		LOG(LERROR)<< "S2OMSequence: file " << name_cloud_xyzrgb
		<< " not found\n";
		return;
	} //: if

	// Read SIFT cloud.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift = pcl::PointCloud<PointXYZSIFT>::Ptr(
			new pcl::PointCloud<PointXYZSIFT>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<PointXYZSIFT>(name_cloud_xyzsift, *cloud_xyzsift) == -1) {
		LOG(LERROR)<< "S2OMSequence: file " << name_cloud_xyzsift
		<< " not found\n";
		return;
	} //: if

	// Read SHOT cloud.
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot = pcl::PointCloud<PointXYZSHOT>::Ptr(
			new pcl::PointCloud<PointXYZSHOT>());
	// Try to load the file.
	if (pcl::io::loadPCDFile<PointXYZSHOT>(name_cloud_xyzshot, *cloud_xyzshot) == -1) {
		LOG(LERROR)<< "S2OMSequence: file " << name_cloud_xyzshot
		<< " not found\n";
		return;
	} //: if

	out_cloud_xyzrgb.write(cloud_xyzrgb);
	out_cloud_xyzsift.write(cloud_xyzsift);
	out_cloud_xyzshot.write(cloud_xyzshot);
	out_name.write(model_name);

}

void S2OMSequence::onLoadImage() {
	CLOG(LDEBUG)<< "Sequence::onLoadImage";

	if (reload_flag) {
		// Try to reload sequence.
		if (!findFiles()) {
			CLOG(LERROR) << name() << ": There are no files .json in "
			<< prop_directory;
		}
		frame = -1;
		reload_flag = false;
	}

	// Check whether there are any images loaded.
	if (files.empty())
	return;

	// Check triggering mode.
	if ((prop_auto_trigger) || (!prop_auto_trigger && next_image_flag)) {
		frame++;
		// Anyway, reset flag.
		next_image_flag = false;

		// Check frame number.
		if (frame < 0)
		frame = 0;
		// Check the size of the dataset.
		if (frame >= files.size()) {
			if (prop_loop) {
				frame = 0;
				CLOG(LINFO) << name() << ": loop";
				// TODO: endOfSequence->raise();
			} else {
				frame = files.size() - 1;
				CLOG(LINFO) << name() << ": end of sequence";
				return;
				// TODO: endOfSequence->raise();
			}

		}

		CLOG(LTRACE) << "Sequence: reading image " << files[frame] << " " << frame + 1 << "/" << files.size();
		readModel(files[frame]);
	}
}

void S2OMSequence::onTriggeredLoadNextImage() {
	CLOG(LDEBUG)<< "Sequence::onTriggeredLoadNextImage - next image from the sequence will be loaded";
	in_trigger.read();
	next_image_flag = true;
}

void S2OMSequence::onLoadNextImage() {
	CLOG(LDEBUG)<< "Sequence::onLoadNextImage - next image from the sequence will be loaded";
	next_image_flag = true;
}

void S2OMSequence::onSequenceReload() {
	CLOG(LDEBUG)<< "Sequence::onSequenceReload";
	reload_flag = true;
}

bool S2OMSequence::onStart() {
	return true;
}

bool S2OMSequence::onStop() {
	return true;
}

bool S2OMSequence::findFiles() {
	files.clear();

	files = Utils::searchFiles(prop_directory, ".*\\.json");

	if (prop_sort)
		std::sort(files.begin(), files.end());

	CLOG(LINFO)<< "Sequence loaded.";
	BOOST_FOREACH(std::string fname, files)
		CLOG(LINFO)<< fname;

	return !files.empty();
}

} //: namespace S2OMSequence
} //: namespace Processors
