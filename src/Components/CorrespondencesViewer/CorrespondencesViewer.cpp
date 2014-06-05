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
	registerStream("in_cloud", &in_cloud);
	registerStream("in_shots", &in_shots);
//	registerStream("in_src", &in_src);
//	registerStream("in_tgt", &in_tgt);
//	registerStream("in_corrs_1", &in_corrs_1);
//	registerStream("in_corrs_2", &in_corrs_2);

	// Register handlers
	h_display.setup(boost::bind(&CorrespondencesViewer::display, this));
	registerHandler("h_display", &h_display);
	addDependency("h_display", &in_cloud);
	addDependency("h_display", &in_shots);
	
	h_on_spin.setup(boost::bind(&CorrespondencesViewer::on_spin, this));
	registerHandler("on_spin", &h_on_spin);
	addDependency("on_spin", NULL);


}

bool CorrespondencesViewer::onInit() {
	viewer = new pcl::visualization::PCLVisualizer ("CorrespondencesViewer");
	viewer->setBackgroundColor (0, 0, 0);
	//viewer->addPointCloud<pcl::PointXYZRGB> (pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>), "sample cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "sample cloud");

	viewer->initCameraParameters ();

	v1 = 0;
	v2 = 1;

	//viewer->createViewPort (0.0, 0.0, 1.0, 1.0, v1);
//	viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);

	viewer->addCoordinateSystem (1.0, 1);
//	viewer->addCoordinateSystem (1.0, v2);

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
	LOG(LTRACE) << "CorrespondencesViewer::display";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud_1_temp = in_cloud.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*in_cloud_1_temp, *cloud_1);

	pcl::PointCloud<PointXYZSHOT>::Ptr shots = in_shots.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*shots, *cloud_2);
	
	std::vector<int> indices;
	cloud_1->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud_1, *cloud_1, indices);

	//pcl::VoxelGrid<pcl::PointXYZ> vg;
	//vg.setLeafSize (0.005, 0.005, 0.005);
	//vg.setInputCloud (cloud_1);
	//vg.filter (*cloud_1);

/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr in_src_temp = in_src.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>(*in_src_temp));

	pcl::PointCloud<pcl::PointXYZ>::Ptr in_tgt_temp = in_tgt.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZ>(*in_tgt_temp));

	pcl::CorrespondencesPtr in_corrs_1_temp = in_corrs_1.read();
	pcl::CorrespondencesPtr corrs_1(new pcl::Correspondences(*in_corrs_1_temp));

	pcl::CorrespondencesPtr in_corrs_2_temp = in_corrs_2.read();
	pcl::CorrespondencesPtr corrs_2(new pcl::Correspondences(*in_corrs_2_temp));

*/
	viewer->removeAllPointClouds();
//	viewer->removeAllPointClouds(v2);
	//viewer->removeAllShapes();
	//viewer->removeAllShapes(v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (cloud_1, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (cloud_2, 255, 0, 0);
	//////
	viewer->addPointCloud<pcl::PointXYZ> (cloud_1, green, "cloud");
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, red, "shots");
	//////
/*	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, red, "new", v1);
	viewer->addPointCloud<pcl::PointXYZ> (cloud_2, red, "new2", v2);

	viewer->addPointCloud<pcl::PointXYZ> (src, "keypoints1_1", v1);
	viewer->addPointCloud<pcl::PointXYZ> (tgt, "keypoints1_2", v2);
	//////
	viewer->addPointCloud<pcl::PointXYZ> (src, "keypoints2_1", v1);
	viewer->addPointCloud<pcl::PointXYZ> (tgt, "keypoints2_2", v2);
*/
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "shots");

	/*viewer->addCorrespondences<pcl::PointXYZ>(src, tgt, *corrs_1, "corrs_1", v1);
	viewer->addCorrespondences<pcl::PointXYZ>(src, tgt, *corrs_2, "corrs_2", v2);
*/
	//viewer->spinOnce (100);
}

void CorrespondencesViewer::on_spin() {
	viewer->spinOnce (100);
}

} //: namespace CorrespondencesViewer
} //: namespace Processors
