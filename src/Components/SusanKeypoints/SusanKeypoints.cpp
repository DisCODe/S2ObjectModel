/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "SusanKeypoints.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/keypoints/susan.h>
#include <pcl/filters/voxel_grid.h>


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace Processors {
namespace SusanKeypoints {

SusanKeypoints::SusanKeypoints(const std::string & name) :
		Base::Component(name)  {

}

SusanKeypoints::~SusanKeypoints() {
}

void SusanKeypoints::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud", &in_cloud);
	registerStream("out_keypoints", &out_keypoints);
	registerStream("out_keypoints_rgb", &out_keypoints_rgb);

	h_compute.setup(boost::bind(&SusanKeypoints::compute, this));
	registerHandler("compute", &h_compute);
	addDependency("compute", &in_cloud);
}

bool SusanKeypoints::onInit() {

	return true;
}

bool SusanKeypoints::onFinish() {
	return true;
}

bool SusanKeypoints::onStop() {
	return true;
}

bool SusanKeypoints::onStart() {
	return true;
}


void SusanKeypoints::compute() {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud.read();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(
			new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::copyPointCloud(*cloud, *copy);

	LOG(LNOTICE)<< "SusanKeypoints: copy size :" << copy->size();
	LOG(LNOTICE)<< "SusanKeypoints: cloud size :" << cloud->size();

	pcl::io::savePCDFileASCII("chmura.pcd", *cloud);

	if (copy->size() > 0) {

		// downsample filtered cloud
/*		  pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
		  voxel_grid.setLeafSize (0.007f, 0.007f, 0.007f);
		  voxel_grid.setInputCloud (copy);
		  voxel_grid.filter (*copy);*/

		CLOG(LNOTICE)<< "SusanKeypoints: : computing keypoints... " << copy->size();

		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

		pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* susan3D = new  pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
		susan3D->setInputCloud(copy);
		susan3D->setNonMaxSupression(false);
		susan3D->setSearchMethod(tree);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
		susan3D->compute(*keypoints);

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_xyz(new pcl::PointCloud<pcl::PointXYZ>());

		pcl::copyPointCloud(*keypoints, *keypoints_xyz);

		CLOG(LNOTICE)<< "SusanKeypoints size :" << keypoints->size();
		out_keypoints_rgb.write(keypoints);
		out_keypoints.write(keypoints_xyz);
	}
}

} //: namespace SusanKeypoints
} //: namespace Processors
