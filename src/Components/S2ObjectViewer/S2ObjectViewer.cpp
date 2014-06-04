/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "S2ObjectViewer.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/filters/filter.h>

namespace Processors {
namespace S2ObjectViewer {

S2ObjectViewer::S2ObjectViewer(const std::string & name) :
		Base::Component(name)  {

}

S2ObjectViewer::~S2ObjectViewer() {
}

void S2ObjectViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud", &in_cloud);
	registerStream("in_shots", &in_shots);
	registerStream("in_sifts", &in_sifts);

	// Register handlers
	h_display.setup(boost::bind(&S2ObjectViewer::display, this));
	registerHandler("h_display", &h_display);
	addDependency("h_display", &in_cloud);
	addDependency("h_display", &in_shots);
	addDependency("h_display", &in_sifts);

	h_on_spin.setup(boost::bind(&S2ObjectViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);
}

bool S2ObjectViewer::onInit() {
	viewer = new pcl::visualization::PCLVisualizer ("S2ObjectViewer");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->initCameraParameters ();
	viewer->addCoordinateSystem (1.0, 1);
	return true;
}

bool S2ObjectViewer::onFinish() {
	return true;
}

bool S2ObjectViewer::onStop() {
	return true;
}

bool S2ObjectViewer::onStart() {
	return true;
}

void S2ObjectViewer::display() {
	LOG(LTRACE) << "S2ObjectViewer::display";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_1_temp = in_cloud.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*in_cloud_1_temp, *cloud_1);

	pcl::PointCloud<PointXYZSHOT>::Ptr shots = in_shots.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*shots, *cloud_2);

	pcl::PointCloud<PointXYZSIFT>::Ptr sifts = in_sifts.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_3(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*sifts, *cloud_3);

	std::vector<int> indices;
	cloud_1->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1, indices);


	viewer->removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (	cloud_1, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (	cloud_2, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (	cloud_3, 255, 0, 0);


	viewer->addPointCloud<pcl::PointXYZ> (cloud_1, blue, "cloud");
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, green, "shots");
	viewer->addPointCloud<pcl::PointXYZ> (cloud_3, red, "sifts");

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sifts");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "shots");
}

void S2ObjectViewer::on_spin() {
	viewer->spinOnce (100);
}



} //: namespace S2ObjectViewer
} //: namespace Processors
