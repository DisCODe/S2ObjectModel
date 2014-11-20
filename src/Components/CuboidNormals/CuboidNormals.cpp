/*!
 * \file
 * \brief
 * \author jkrasnod
 */

#include <memory>
#include <string>

#include "CuboidNormals.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>

namespace Processors {
namespace CuboidNormals {

CuboidNormals::CuboidNormals(const std::string & name) :
		Base::Component(name), radius("radius", 0.05) {
	registerProperty(radius);

}

CuboidNormals::~CuboidNormals() {
}

void CuboidNormals::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_normals", &out_cloud_normals);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);
//	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyz.setup(boost::bind(&CuboidNormals::computeNormalsXYZ, this));
	registerHandler("h_compute_xyz", &h_compute_xyz);
	addDependency("h_compute_xyz", &in_cloud_xyz);

	h_compute_xyzrgb.setup(boost::bind(&CuboidNormals::computeNormalsXYZRGB, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);

}

bool CuboidNormals::onInit() {

	return true;
}

bool CuboidNormals::onFinish() {
	return true;
}

bool CuboidNormals::onStop() {
	return true;
}

bool CuboidNormals::onStart() {
	return true;
}

void CuboidNormals::computeNormalsXYZ() {
	LOG(LTRACE) << "CuboidNormals::computeNormalsXYZ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);


	// compute centroid

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*copy,centroid);

	// compute normals for centroid as viewpoint

	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	  normalEstimation.setInputCloud (copy);
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	  normalEstimation.setSearchMethod (tree);

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  normalEstimation.setRadiusSearch (radius);
	  normalEstimation.setViewPoint(centroid[0], centroid[1], centroid[2]);
	  normalEstimation.compute (*cloud_normals);

	  // flip normlas orientation

	  for (int i = 0; i < cloud_normals->size(); ++i) {
		  cloud_normals->points[i].normal_x = - cloud_normals->points[i].normal_x;
		  cloud_normals->points[i].normal_y = - cloud_normals->points[i].normal_y;
		  cloud_normals->points[i].normal_z = - cloud_normals->points[i].normal_z;
	  }

//	  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_xyz_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
//	  pcl::copyPointCloud(*copy, *cloud_xyz_normals);
//	  pcl::copyPointCloud(*cloud_normals, *cloud_xyz_normals);
//
	//  out_cloud_xyz_normals.write(cloud_xyz_normals);
	  out_cloud_normals.write(cloud_normals);
}

void CuboidNormals::computeNormalsXYZRGB() {
	LOG(LTRACE) << "CuboidNormals::computeNormalsXYZRGB";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	LOG(LTRACE) << "CuboidNormals: input size: " << cloud->size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());



	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	LOG(LTRACE) << "CuboidNormals: input size: " << copy->size();

	// compute centroid

	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*copy,centroid);
	LOG(LDEBUG) << "CuboidNormals: centroid computed: " << centroid;

	// compute normals for centroid as viewpoint

	  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	  normalEstimation.setInputCloud (copy);
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	  normalEstimation.setSearchMethod (tree);

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  normalEstimation.setRadiusSearch (radius);
	  normalEstimation.setViewPoint(centroid[0], centroid[1], centroid[2]);

	  LOG(LDEBUG) << "CuboidNormals: compute normals!";
	  normalEstimation.compute (*cloud_normals);

	  // flip normlas orientation
	  LOG(LTRACE) << "CuboidNormals: flip normals!";


	  for (int i = 0; i < cloud_normals->size(); ++i) {
		  cloud_normals->points[i].normal_x = - cloud_normals->points[i].normal_x;
		  cloud_normals->points[i].normal_y = - cloud_normals->points[i].normal_y;
		  cloud_normals->points[i].normal_z = - cloud_normals->points[i].normal_z;
	  }


	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyz_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  pcl::copyPointCloud(*copy, *cloud_xyz_normals);
	  pcl::copyPointCloud(*cloud_normals, *cloud_xyz_normals);

	  out_cloud_xyzrgb_normals.write(cloud_xyz_normals);

	  LOG(LTRACE) << "CuboidNormals: sending normals! " << cloud_normals->size();
	  out_cloud_normals.write(cloud_normals);
}



} //: namespace CuboidNormals
} //: namespace Processors
