/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "CorrespondencesViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name),
		display_source_cloud("display.source_cloud", true),
		display_target_cloud("display.target_cloud", true),
		display_target_keypoints("display.target_keypoints",true),
		display_source_keypoints("display.source_keypoints", true) {

	registerProperty(display_source_cloud);
	registerProperty(display_target_cloud);
	registerProperty(display_target_keypoints);
	registerProperty(display_source_keypoints);

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
	h_display.setup(boost::bind(&CorrespondencesViewer::display, this));
	registerHandler("h_display", &h_display);
	addDependency("h_display", &in_correspondeces);
	addDependency("h_display", &in_source_keypoints);
	addDependency("h_display", &in_target_keypoints);
	addDependency("h_display", &in_source_cloud);
	addDependency("h_display", &in_target_cloud);

	h_on_spin.setup(boost::bind(&CorrespondencesViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);

}

bool CorrespondencesViewer::onInit() {
	viewer = new pcl::visualization::PCLVisualizer ("S2ObjectModel: CorrespondencesViewer");
	viewer->setBackgroundColor (255, 255, 255);
	viewer->initCameraParameters ();
	//viewer->addCoordinateSystem (1.0, 1);
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
	LOG(LWARNING) << "CorrespondencesViewer::display";

	pcl::CorrespondencesPtr corrs = in_correspondeces.read();

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_temp = in_source_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*source_keypoints_temp));
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_temp = in_target_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*target_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_temp = in_source_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*source_cloud_temp));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_temp = in_target_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*target_cloud_temp));


	std::vector<int> indices;
	source_keypoints_temp->is_dense = false;
	pcl::removeNaNFromPointCloud(*source_keypoints_temp, *source_keypoints, indices);

	indices.clear();
	target_keypoints_temp->is_dense = false;
	pcl::removeNaNFromPointCloud(*target_keypoints_temp, *target_keypoints, indices);

	indices.clear();
	source_cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*source_cloud_temp, *source_cloud, indices);

	indices.clear();
	target_cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*target_cloud_temp, *target_cloud, indices);

	if (source_keypoints_temp->size() != source_keypoints->size()) {
		CLOG(LERROR) << "problem[source]!: " << source_keypoints_temp->size() << " != " << source_keypoints->size();
	}
	if (target_keypoints_temp->size() != target_keypoints->size()) {
		CLOG(LERROR) << "problem[target]!: " << target_keypoints_temp->size() << " != " << target_keypoints->size();
	}


	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sourceRgb(source_cloud);
	if (display_source_cloud) viewer->addPointCloud<pcl::PointXYZRGB> (source_cloud, sourceRgb, "source_cloud");

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> targetRgb(target_cloud);
	if (display_target_cloud) viewer->addPointCloud<pcl::PointXYZRGB> (target_cloud, targetRgb, "target_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (target_keypoints, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (source_keypoints, 255, 0, 0);


	if (display_target_keypoints) viewer->addPointCloud<pcl::PointXYZ> (target_keypoints, green, "target_keypoints");
	if (display_source_keypoints) viewer->addPointCloud<pcl::PointXYZ> (source_keypoints, red, "source_keypoints");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_keypoints");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_keypoints");

	viewer->addCorrespondences<pcl::PointXYZ>(source_keypoints, target_keypoints, *corrs);

/*	pcl::CorrespondencesPtr corrs_temp = in_correspondeces.read();
	pcl::CorrespondencesPtr corrs(new pcl::Correspondences(*corrs_temp));

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_temp = in_source_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*source_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints_temp = in_target_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints( new pcl::PointCloud<pcl::PointXYZ>(*target_keypoints_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud_temp = in_source_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*source_cloud_temp));

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_temp = in_target_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud( new pcl::PointCloud<pcl::PointXYZRGB>(*target_cloud_temp));



	LOG(LWARNING) << "CorrespondencesViewer::source_keypoints size: " << source_keypoints->size();
	LOG(LWARNING) << "CorrespondencesViewer::target_keypoints size: " << target_keypoints->size();

	std::vector<int> indices;
	source_cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*source_cloud, *source_cloud, indices);
	LOG(LWARNING) << "CorrespondencesViewer::source_cloud size: " << source_cloud->size();

	std::vector<int> indices2;
	source_cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*target_cloud, *target_cloud, indices2);
	LOG(LWARNING) << "CorrespondencesViewer::target_cloud size: " << target_cloud->size();
	LOG(LWARNING) << "CorrespondencesViewer::corrs size: " << corrs->size();

	for (int i = 0; i < corrs->size(); ++i) {
		pcl::Correspondence corr = corrs->at(i);
		if (corr.index_query >= source_keypoints->size()) {
			CLOG(LERROR) << "source_keypoints size : " << source_keypoints->size() << ", index: " << corr.index_query;
		}
		if (corr.index_match >= target_keypoints->size()) {
			CLOG(LERROR) << "target_keypoints size : " << target_keypoints->size() << ", index: " << corr.index_match;
		}
	}

	LOG(LWARNING) << "CorrespondencesViewer::corrs ok!";

	LOG(LWARNING) << "CorrespondencesViewer::removeAllPointClouds";
	viewer->removeAllPointClouds();
	LOG(LWARNING) << "CorrespondencesViewer::removeAllShapes";
	viewer->removeAllShapes();

	LOG(LWARNING) << "CorrespondencesViewer::addPointCloud(source_cloud)";
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sourceRgb(source_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (source_cloud, sourceRgb, "source_cloud");

	LOG(LWARNING) << "CorrespondencesViewer::addPointCloud(target_cloud)";
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> targetRgb(target_cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (target_cloud, sourceRgb, "target_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (target_keypoints, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (source_keypoints, 255, 0, 0);


	LOG(LWARNING) << "CorrespondencesViewer::addPointCloud(target_keypoints)";
	viewer->addPointCloud<pcl::PointXYZ> (target_keypoints, green, "target_keypoints");
	LOG(LWARNING) << "CorrespondencesViewer::addPointCloud(source_keypoints)";
	viewer->addPointCloud<pcl::PointXYZ> (source_keypoints, red, "source_keypoints");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "target_keypoints");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "source_keypoints");

	LOG(LWARNING) << "CorrespondencesViewer::addCorrespondences";
	viewer->addCorrespondences<pcl::PointXYZ>(source_keypoints, target_keypoints, *corrs);
	LOG(LWARNING) << "CorrespondencesViewer::ok!";*/
}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}


} //: namespace CorrespondencesViewer
} //: namespace Processors
