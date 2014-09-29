/*!
 * \file
 * \brief
 * \author Joanna,,,
 */

#include <memory>
#include <string>

#include "Normals.hpp"
#include "Common/Logger.hpp"

#include <boost/bind.hpp>

#include <boost/bind.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>


namespace Processors {
namespace Normals {

Normals::Normals(const std::string & name) :
				Base::Component(name), radius("radius", 0.05) {
			registerProperty(radius);
}

Normals::~Normals() {
}

void Normals::prepareInterface() {
	// Register data streams, events and event handlers HERE!
	registerStream("in_cloud_xyz", &in_cloud_xyz);
	registerStream("in_cloud_xyzrgb", &in_cloud_xyzrgb);
	registerStream("out_cloud_normals", &out_cloud_normals);
	registerStream("out_cloud_xyzrgb_normals", &out_cloud_xyzrgb_normals);
//	registerStream("out_cloud_xyzrgb", &out_cloud_xyzrgb);

	h_compute_xyz.setup(boost::bind(&Normals::computeNormalsXYZ, this));
	registerHandler("h_compute_xyz", &h_compute_xyz);
	addDependency("h_compute_xyz", &in_cloud_xyz);

	h_compute_xyzrgb.setup(boost::bind(&Normals::computeNormalsXYZRGB, this));
	registerHandler("h_compute_xyzrgb", &h_compute_xyzrgb);
	addDependency("h_compute_xyzrgb", &in_cloud_xyzrgb);
}

bool Normals::onInit() {

	return true;
}

bool Normals::onFinish() {
	return true;
}

bool Normals::onStop() {
	return true;
}

bool Normals::onStart() {
	return true;
}

void Normals::computeNormalsXYZ() {
	LOG(LWARNING) << "Normals::computeNormalsXYZ";
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = in_cloud_xyz.read();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>());

	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);


	// compute centroid

//	Eigen::Vector4f centroid;
//	pcl::compute3DCentroid(*copy,centroid);

	// compute normals for centroid as viewpoint

	  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	  normalEstimation.setInputCloud (copy);
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	  normalEstimation.setSearchMethod (tree);

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  normalEstimation.setRadiusSearch (radius);
//	  normalEstimation.setViewPoint(centroid[0], centroid[1], centroid[2]);
	  normalEstimation.compute (*cloud_normals);


//	  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_xyz_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
//	  pcl::copyPointCloud(*copy, *cloud_xyz_normals);
//	  pcl::copyPointCloud(*cloud_normals, *cloud_xyz_normals);
//
	//  out_cloud_xyz_normals.write(cloud_xyz_normals);
	  out_cloud_normals.write(cloud_normals);
}

void Normals::computeNormalsXYZRGB() {
	LOG(LWARNING) << "Normals::computeNormalsXYZRGB";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = in_cloud_xyzrgb.read();

	LOG(LWARNING) << "Normals: input size: " << cloud->size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr copy(new pcl::PointCloud<pcl::PointXYZRGB>());



	// Remove NaNs.
	std::vector<int> indicesNANs;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *copy, indicesNANs);

	LOG(LWARNING) << "Normals: input size: " << copy->size();

	// compute centroid

//	Eigen::Vector4f centroid;
//	pcl::compute3DCentroid(*copy,centroid);
	//LOG(LDEBUG) << "Normals: centroid computed: " << centroid;

	// compute normals for centroid as viewpoint

	  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
	  normalEstimation.setInputCloud (copy);
	  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	  normalEstimation.setSearchMethod (tree);

	  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	  normalEstimation.setRadiusSearch (radius);
//	  normalEstimation.setViewPoint(centroid[0], centroid[1], centroid[2]);

	  LOG(LWARNING) << "Normals: compute normals!";
	  normalEstimation.compute (*cloud_normals);



	  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyz_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	  pcl::copyPointCloud(*copy, *cloud_xyz_normals);
	  pcl::copyPointCloud(*cloud_normals, *cloud_xyz_normals);

	  LOG(LWARNING) << "Normals: out_cloud_xyzrgb_normals.write " << cloud_normals->size();
	  out_cloud_xyzrgb_normals.write(cloud_xyz_normals);

	  LOG(LWARNING) << "Normals: out_cloud_normals.write " << cloud_normals->size();
	  out_cloud_normals.write(cloud_normals);
}

} //: namespace Normals
} //: namespace Processors
