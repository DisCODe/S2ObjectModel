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


#include <pcl/filters/voxel_grid.h>

namespace Processors {
namespace CorrespondencesViewer {

CorrespondencesViewer::CorrespondencesViewer(const std::string & name) :
		Base::Component(name)  {

}

CorrespondencesViewer::~CorrespondencesViewer() {
}

void CorrespondencesViewer::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_1", &in_cloud_1);
	registerStream("in_cloud_2", &in_cloud_2);
	registerStream("in_src", &in_src);
	registerStream("in_tgt", &in_tgt);
	registerStream("in_corrs_1", &in_corrs_1);
	registerStream("in_corrs_2", &in_corrs_2);

	// Register handlers
	h_display.setup(boost::bind(&CorrespondencesViewer::display, this));
	registerHandler("h_display", &h_display);
	addDependency("h_display", &in_cloud_1);
	addDependency("h_display", &in_cloud_2);
	addDependency("h_display", &in_src);
	addDependency("h_display", &in_tgt);
	addDependency("h_display", &in_corrs_1);
	addDependency("h_display", &in_corrs_2);

}

bool CorrespondencesViewer::onInit() {
	viewer = new pcl::visualization::PCLVisualizer ("CorrespondencesViewer");
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>), "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");

	viewer->initCameraParameters ();

	v1 = 0;
	v2 = 1;

	viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);

	viewer->addCoordinateSystem (1.0, v1);
	viewer->addCoordinateSystem (1.0, v2);

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

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_1_temp = in_cloud_1.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>(*in_cloud_1_temp));

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_2_temp = in_cloud_2.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>(*in_cloud_2_temp));

	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize (0.005, 0.005, 0.005);

	vg.setInputCloud (in_cloud_1_temp);
	vg.filter (*cloud_1);

	vg.setInputCloud (in_cloud_2_temp);
	vg.filter (*cloud_2);


	pcl::PointCloud<pcl::PointXYZ>::Ptr in_src_temp = in_src.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(*in_src_temp));

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_tgt_temp = in_tgt.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(*in_tgt_temp));

	pcl::CorrespondencesPtr in_corrs_1_temp = in_corrs_1.read();
	pcl::CorrespondencesPtr corrs_1(new pcl::Correspondences(*in_corrs_1_temp));

	pcl::CorrespondencesPtr in_corrs_2_temp = in_corrs_2.read();
	pcl::CorrespondencesPtr corrs_2(new pcl::Correspondences(*in_corrs_2_temp));

	viewer->removeAllPointClouds(v1);
	viewer->removeAllPointClouds(v2);
	viewer->removeAllShapes(v1);
	viewer->removeAllShapes(v2);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_1, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_1, 255, 0, 0);
	//////
	viewer->addPointCloud<pcl::PointXYZ> (cloud_1, green, "merged", v1);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_1, green, "merged2", v2);
	//////
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, red, "new", v1);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, red, "new2", v2);

	viewer->addPointCloud<pcl::PointXYZ> (src, "keypoints1_1", v1);
	viewer->addPointCloud<pcl::PointXYZ> (tgt, "keypoints1_2", v2);
	//////
	viewer->addPointCloud<pcl::PointXYZ> (src, "keypoints2_1", v1);
	viewer->addPointCloud<pcl::PointXYZ> (tgt, "keypoints2_1", v2);

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints1_1", v1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints1_2", v2);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints2_1", v1);
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints2_1", v2);

	viewer->addCorrespondences<pcl::PointXYZ>(src, tgt, *corrs_1, "corrs_1", v1);
	viewer->addCorrespondences<pcl::PointXYZ>(src, tgt, *corrs_2, "corrs_2", v2);

	viewer->spinOnce (100);
}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}

} //: namespace CorrespondencesViewer
} //: namespace Processors
