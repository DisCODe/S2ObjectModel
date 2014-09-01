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

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name)  {

}

CorrespondencesViewer::~CorrespondencesViewer() {
}

void CorrespondencesViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_source_keypoints", &in_source_keypoints);
	registerStream("in_source", &in_source);
	registerStream("in_target_keypoints", &in_target_keypoints);
	registerStream("in_target", &in_target);
	registerStream("in_correspondences", &in_correspondences);


	// Register handlers
	h_display.setup(boost::bind(&CorrespondencesViewer::display, this));
	registerHandler("h_display", &h_display);
	addDependency("h_display", &in_source_keypoints);
	addDependency("h_display", &in_source);
	addDependency("h_display", &in_target_keypoints);
	addDependency("h_display", &in_target);
	addDependency("h_display", &in_correspondences);

	h_on_spin.setup(boost::bind(&CorrespondencesViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool CorrespondencesViewer::onInit() {
	viewer = new pcl::visualization::PCLVisualizer ("CorrespondencesViewer");
	viewer->initCameraParameters ();
	viewer->setBackgroundColor(255, 255, 255);
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
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_source_keypoints = in_source_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_target_keypoints = in_target_keypoints.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_source = in_source.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_target = in_target.read();
	pcl::CorrespondencesPtr corrs = in_correspondences.read();

 /*   Eigen::Affine3f t;
    pcl::getTransformation(10.0,5.0,20.0,0.0,0.0,0.0,t);
    */
	Eigen::Matrix4f t = Eigen::Matrix4f::Identity() ;
	float tx = 0.3f, ty = 0.0f, tz = 0.0f ;
	t(0, 3) = tx ; t(1, 3) = ty ; t(2, 3) = tz ;

    pcl::transformPointCloud(*temp_target_keypoints, *temp_target_keypoints, t);
    pcl::transformPointCloud(*temp_target, *temp_target, t);

	viewer->removeAllPointClouds();
	viewer->removeAllShapes();

	LOG(LTRACE) << "S2ObjectViewer::display 10";
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> source_rgb(temp_source);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> target_rgb(temp_target);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (	temp_source_keypoints, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (	temp_target_keypoints, 255, 0, 0);

	LOG(LTRACE) << "S2ObjectViewer::display 11";

	viewer->addPointCloud<pcl::PointXYZRGB> (temp_source, source_rgb, "source");
	viewer->addPointCloud<pcl::PointXYZRGB> (temp_target, target_rgb, "target");
	viewer->addPointCloud<pcl::PointXYZ> (temp_source_keypoints, green, "keypoints_source");
	viewer->addPointCloud<pcl::PointXYZ> (temp_target_keypoints, red, "keypoints_target");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "keypoints_source");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints_target");

	viewer->addCorrespondences<pcl::PointXYZ>(temp_source_keypoints, temp_target_keypoints, *corrs, "correspondences") ;
	viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
		0,
		0,
		255,
		"correspondences") ;
}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}

} //: namespace CorrespondencesViewer
} //: namespace Processors
