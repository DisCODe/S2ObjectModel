/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "KeypointsViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/filters/filter.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Processors {
namespace KeypointsViewer {

KeypointsViewer::KeypointsViewer(const std::string & name) :
		Base::Component(name)  {

}

KeypointsViewer::~KeypointsViewer() {
}

void KeypointsViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud", &in_cloud);
	registerStream("in_keypoints", &in_keypoints);


	// Register handlers
	h_display.setup(boost::bind(&KeypointsViewer::display, this));
	registerHandler("h_display", &h_display);
	addDependency("h_display", &in_cloud);
	addDependency("h_display", &in_keypoints);

	h_on_spin.setup(boost::bind(&KeypointsViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);

}

bool KeypointsViewer::onInit() {
	viewer = new pcl::visualization::PCLVisualizer ("KeypointsViewer");

	left = int(1);
	left = int(2);

	viewer->createViewPort (0.0, 0.0, 0.5, 1.0, left);
	viewer->createViewPort (0.5, 0.0, 1.0, 1.0, right);
	viewer->initCameraParameters ();
//	viewer->addCoordinateSystem (1.0, "test", 1);
	return true;
}

bool KeypointsViewer::onFinish() {
	return true;
}

bool KeypointsViewer::onStop() {
	return true;
}

bool KeypointsViewer::onStart() {
	return true;
}

void KeypointsViewer::display() {
	LOG(LWARNING) << "S2ObjectViewer::display";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_1_temp = in_cloud.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*in_cloud_1_temp, *cloud_1);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_2_temp = in_keypoints.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*in_cloud_2_temp, *cloud_2);

	std::vector<int> indices;
	cloud_1->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1, indices);

	std::vector<int> indices2;
	cloud_1->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_2, *cloud_2, indices2);

	viewer->removeAllPointClouds();

	LOG(LTRACE) << "S2ObjectViewer::display 10";
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(in_cloud_1_temp);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (	cloud_1, 0, 200, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (	cloud_2, 255, 0, 0);

	LOG(LTRACE) << "S2ObjectViewer::display 11";

	viewer->addPointCloud<pcl::PointXYZRGB> (in_cloud_1_temp, rgb, "orginal", right);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_1, green, "cloud", left);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, red, "keypoints", left);

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");
}

void KeypointsViewer::on_spin() {
	viewer->spinOnce (100);
}

} //: namespace KeypointsViewer
} //: namespace Processors
