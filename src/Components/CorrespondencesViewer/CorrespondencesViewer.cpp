/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>
#include <sstream>

#include "CorrespondencesViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name), display_source_cloud("display.source_cloud", true), display_target_cloud(
				"display.target_cloud", true), display_target_keypoints("display.target_keypoints", true), display_source_keypoints(
				"display.source_keypoints", true), correspondences_size("correspondences.size", 1), all_viewpoints(
				"display.all_viewpoints", false) {

	registerProperty(display_source_cloud);
	registerProperty(display_target_cloud);
	registerProperty(display_target_keypoints);
	registerProperty(display_source_keypoints);
	registerProperty(all_viewpoints);
	registerProperty(correspondences_size);

}

CorrespondencesViewer::~CorrespondencesViewer() {
}

void CorrespondencesViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_correspondeces", &in_correspondeces);
	registerStream("in_source_keypoints", &in_source_keypoints);
	registerStream("in_target_keypoints", &in_target_keypoints);
	registerStream("in_source_cloud", &in_source_cloud);
	registerStream("in_target_cloud", &in_target_cloud);

	// Register handlers
	registerHandler("h_display", boost::bind(&CorrespondencesViewer::display, this));
	addDependency("h_display", &in_correspondeces);
	addDependency("h_display", &in_source_keypoints);
	addDependency("h_display", &in_target_keypoints);
	addDependency("h_display", &in_source_cloud);
	addDependency("h_display", &in_target_cloud);

	// Register data streams, events and event handlers HERE!
	registerStream("in_correspondeces2", &in_correspondeces2);
	registerStream("in_source_keypoints2", &in_source_keypoints2);
	registerStream("in_target_keypoints2", &in_target_keypoints2);
	registerStream("in_source_cloud2", &in_source_cloud2);
	registerStream("in_target_cloud2", &in_target_cloud2);

	// Register handlers
	registerHandler("h_display2", boost::bind(&CorrespondencesViewer::display2, this));
	addDependency("h_display2", &in_correspondeces2);
	addDependency("h_display2", &in_source_keypoints2);
	addDependency("h_display2", &in_target_keypoints2);
	addDependency("h_display2", &in_source_cloud2);
	addDependency("h_display2", &in_target_cloud2);

	// Register data streams, events and event handlers HERE!
	registerStream("in_correspondeces3", &in_correspondeces3);
	registerStream("in_source_keypoints3", &in_source_keypoints3);
	registerStream("in_target_keypoints3", &in_target_keypoints3);
	registerStream("in_source_cloud3", &in_source_cloud3);
	registerStream("in_target_cloud3", &in_target_cloud3);

	// Register handlers
	registerHandler("h_display3", boost::bind(&CorrespondencesViewer::display3, this));
	addDependency("h_display3", &in_correspondeces3);
	addDependency("h_display3", &in_source_keypoints3);
	addDependency("h_display3", &in_target_keypoints3);
	addDependency("h_display3", &in_source_cloud3);
	addDependency("h_display3", &in_target_cloud3);

	// Register data streams, events and event handlers HERE!
	registerStream("in_correspondeces4", &in_correspondeces4);
	registerStream("in_source_keypoints4", &in_source_keypoints4);
	registerStream("in_target_keypoints4", &in_target_keypoints4);
	registerStream("in_source_cloud4", &in_source_cloud4);
	registerStream("in_target_cloud4", &in_target_cloud4);

	// Register handlers
	registerHandler("h_display4", boost::bind(&CorrespondencesViewer::display4, this));
	addDependency("h_display4", &in_correspondeces4);
	addDependency("h_display4", &in_source_keypoints4);
	addDependency("h_display4", &in_target_keypoints4);
	addDependency("h_display4", &in_source_cloud4);
	addDependency("h_display4", &in_target_cloud4);

	registerHandler("on_spin", boost::bind(&CorrespondencesViewer::on_spin, this));
	addDependency("on_spin", NULL);

}

bool CorrespondencesViewer::onInit() {
	/*	viewer = new pcl::visualization::PCLVisualizer ("S2ObjectModel: CorrespondencesViewer");
	 viewer->setBackgroundColor (255, 255, 255);
	 viewer->initCameraParameters ();*/
	//viewer->addCoordinateSystem (1.0, 1);
	left = int(1);
	right = int(2);
	viewer = new pcl::visualization::PCLVisualizer("S2ObjectModel: CorrespondencesViewer");
	viewer->initCameraParameters();

	if (all_viewpoints) {
		CLOG(LTRACE)<< "CorrespondencesViewer::onInit : create 4 viewports";
		left_down = int(3);
		right_down = int(4);

		viewer->createViewPort (0.0, 0.0, 0.5, 0.5, left_down);
		viewer->createViewPort (0.5, 0.0, 1.0, 0.5, right_down);

		viewer->createViewPort (0.0, 0.5, 0.5, 1.0, left);
		viewer->createViewPort (0.5, 0.5, 1.0, 1.0, right);

		viewer->setBackgroundColor(255, 255, 255, left_down);
		viewer->setBackgroundColor(255, 255, 255, right_down);
		viewer->setBackgroundColor(255, 255, 255, left);
		viewer->setBackgroundColor(255, 255, 255, right);
	} else {
		viewer->createViewPort (0.0, 0.0, 0.5, 1.0, left);
		viewer->createViewPort (0.5, 0.0, 1.0, 1.0, right);

		viewer->setBackgroundColor(255, 255, 255, left);
		viewer->setBackgroundColor(255, 255, 255, right);
	}

	return true;
}

bool CorrespondencesViewer::onFinish() {
	return true;
}

bool CorrespondencesViewer::onStop() {
	return true;
}

bool CorrespondencesViewer::onStart() {
	return true;
}

void CorrespondencesViewer::display() {
	LOG(LTRACE)<< "CorrespondencesViewer::display";

	pcl::CorrespondencesPtr corrs = in_correspondeces.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_temp = in_source_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*source_keypoints_temp));
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_temp = in_target_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*target_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_temp = in_source_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*source_cloud_temp));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_temp = in_target_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*target_cloud_temp));

	displayOnPort(left, source_keypoints, target_keypoints, corrs, source_cloud, target_cloud);
	viewer->addText("SHOT", 5, 5, "left_text", left);
}

void CorrespondencesViewer::display2() {
	LOG(LTRACE)<< "CorrespondencesViewer::display2";

	pcl::CorrespondencesPtr corrs = in_correspondeces2.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_temp = in_source_keypoints2.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*source_keypoints_temp));
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_temp = in_target_keypoints2.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*target_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_temp = in_source_cloud2.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*source_cloud_temp));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_temp = in_target_cloud2.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*target_cloud_temp));

	displayOnPort(right, source_keypoints, target_keypoints, corrs, source_cloud, target_cloud);
	viewer->addText("SIFT", 5, 5, "right_text", right);
}

void CorrespondencesViewer::display3() {
	LOG(LTRACE)<< "CorrespondencesViewer::display3";


	pcl::CorrespondencesPtr corrs = in_correspondeces3.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_temp = in_source_keypoints3.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*source_keypoints_temp));
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_temp = in_target_keypoints3.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*target_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_temp = in_source_cloud3.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*source_cloud_temp));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_temp = in_target_cloud3.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*target_cloud_temp));

    if (!all_viewpoints) return;
	displayOnPort(left_down, source_keypoints, target_keypoints, corrs, source_cloud, target_cloud);

	viewer->addText("wspolny RANSAC", 5, 5, "left_down_text", left_down);
}

void CorrespondencesViewer::display4() {
	LOG(LTRACE)<< "CorrespondencesViewer::display4";


	pcl::CorrespondencesPtr corrs = in_correspondeces4.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_temp = in_source_keypoints4.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*source_keypoints_temp));
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_temp = in_target_keypoints4.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*target_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_temp = in_source_cloud4.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*source_cloud_temp));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_temp = in_target_cloud4.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*target_cloud_temp));

    if (!all_viewpoints) return;
	displayOnPort(right_down, source_keypoints, target_keypoints, corrs, source_cloud, target_cloud);
	viewer->addText("AVG", 5, 5, "right_down_text", right_down);
}

void CorrespondencesViewer::displayOnPort(int port, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
		pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints, pcl::CorrespondencesPtr corrs,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud) {

	std::vector<int> indices;
	source_keypoints->is_dense = false;
	pcl::removeNaNFromPointCloud(*source_keypoints, *source_keypoints, indices);

	indices.clear();
	target_keypoints->is_dense = false;
	pcl::removeNaNFromPointCloud(*target_keypoints, *target_keypoints, indices);

	indices.clear();
	source_cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);

	indices.clear();
	target_cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices);

	viewer->removeAllPointClouds(port);
	viewer->removeAllShapes(port);

	std::string portString = "";

	std::ostringstream numberS;
	numberS << port;
	portString += numberS.str();

	std::string source_cloud_name = "source_cloud" + portString;
	std::string target_cloud_name = "target_cloud" + portString;
	std::string target_keypoints_name = "target_keypoints" + portString;
	std::string source_keypoints_name = "source_keypoints" + portString;
	std::string corrs_name = "corrs" + portString;

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sourceRgb(source_cloud);
	if (display_source_cloud)
		viewer->addPointCloud<pcl::PointXYZRGB>(source_cloud, sourceRgb, source_cloud_name, port);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> targetRgb(target_cloud);
	if (display_target_cloud)
		viewer->addPointCloud<pcl::PointXYZRGB>(target_cloud, targetRgb, target_cloud_name, port);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(target_keypoints, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(source_keypoints, 255, 0, 0);

	if (display_target_keypoints)
		viewer->addPointCloud<pcl::PointXYZ>(target_keypoints, green, target_keypoints_name, port);
	if (display_source_keypoints)
		viewer->addPointCloud<pcl::PointXYZ>(source_keypoints, red, source_keypoints_name, port);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, target_keypoints_name,
			port);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, source_keypoints_name,
			port);

	//viewer->addCorrespondences<pcl::PointXYZ>(source_keypoints, target_keypoints, *corrs, corrs_name, port);

	for (size_t j = 0; j < corrs->size(); ++j) {
		pcl::Correspondence corr = corrs->at(j);

		std::stringstream ss_line;
		ss_line << "correspondence_line" << port << "_" << corr.index_match << "_" << corr.index_query;

		//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
		viewer->addLine<pcl::PointXYZ, pcl::PointXYZ>(source_keypoints->at(corr.index_query),
				target_keypoints->at(corr.index_match), 0, 0, 255, ss_line.str(), port);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, correspondences_size,
				ss_line.str(), port);
	}

}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce(100);
}

} //: namespace CorrespondencesViewer
} //: namespace Processors
