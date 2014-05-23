/*!
 * \file
 * \brief
 * \author jkrasnod
 */

#include <memory>
#include <string>

#include "SHOTConverter.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/sift_keypoint.h>

#include <iostream>

#include <pcl/io/pcd_io.h>

namespace Processors {
namespace SHOTConverter {

SHOTConverter::SHOTConverter(const std::string & name) :
		Base::Component(name), normal_radius("normal_radius", 0.02), shot_radius("shot_radius", 0.05) {
	registerProperty(normal_radius);
	registerProperty(shot_radius);

}

SHOTConverter::~SHOTConverter() {
}

void SHOTConverter::prepareInterface() {

	// Register data streams, events and event handlers HERE!
	registerStream("in_keypoints", &in_keypoints);
	registerStream("in_points", &in_points);
	registerStream("out_shots", &out_shots);
	registerStream("out_cloud_xyzshot", &out_cloud_xyzshot);
	// Register handlers
	h_process.setup(boost::bind(&SHOTConverter::process, this));
	registerHandler("process", &h_process);
	addDependency("process", &in_points);
	addDependency("process", &in_keypoints);

}

bool SHOTConverter::onInit() {

	return true;
}

bool SHOTConverter::onFinish() {
	return true;
}

bool SHOTConverter::onStop() {
	return true;
}

bool SHOTConverter::onStart() {
	return true;
}

NormalCloudPtr SHOTConverter::getNormals(PointCloudPtr cloud) {

	NormalCloudPtr normals(new NormalCloud());
	pcl::NormalEstimationOMP<PointT, NormalT> est;
	est.setRadiusSearch(normal_radius);
	est.setInputCloud(cloud);
	est.compute(*normals);
	return normals;
}

SHOTCloudPtr SHOTConverter::getSHOT(PointCloudPtr cloud, NormalCloudPtr normals, PointCloudPtr keypoints) {
	SHOTCloudPtr shot(new SHOTCloud());

	pcl::SHOTEstimationOMP<PointT, NormalT, SHOT> est;
	est.setRadiusSearch(shot_radius);
	est.setInputCloud(keypoints);
	est.setInputNormals(normals);
	est.setSearchSurface(cloud);
	est.compute(*shot);
	return shot;
}

void SHOTConverter::process() {

	PointCloudPtr cloud = in_points.read();
//	PointCloudPtr cloud(new PointCloud(*temp));

	PointCloudPtr keypoints = in_keypoints.read();
//	PointCloudPtr keypoints(new PointCloud(*temp2));

	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	CLOG(LNOTICE)<< "SHOTConverter: point cloud size (no NANs) : " << cloud->size() << ", keypoints : " << keypoints->size();

	NormalCloudPtr normals = getNormals(cloud);
	SHOTCloudPtr shotCloud = getSHOT(cloud, normals, keypoints);

	XYZSHOTCloudPtr xyzshotcloud(new XYZSHOTCloud());
	pcl::copyPointCloud(*shotCloud, *xyzshotcloud);
	pcl::copyPointCloud(*keypoints, *xyzshotcloud);

//	CLOG(LWARNING)<< "SHOTConverter: saving files...!";
//	pcl::io::savePCDFileASCII("keypoints.pcd", *keypoints);
//	pcl::io::savePCDFileASCII("shots.pcd", *shotCloud);
//	pcl::io::savePCDFileASCII("shot_xyz.pcd", *xyzshotcloud);

	// TODO set multiplicity, pointId

	out_shots.write(shotCloud);
	out_cloud_xyzshot.write(xyzshotcloud);
}

} //: namespace SHOTConverter
} //: namespace Processors
