#ifndef S2OBJECTMODEL_HPP_
#define S2OBJECTMODEL_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Types/PointCloudObject.hpp> 
#include <Types/PointXYZSIFT.hpp> 
#include <Types/PointXYZSHOT.hpp> 
//namespace Types {

/*!
 * \class SIFTObjectModel
 * \brief Model of 3D object.
 * It consists of: object point cloud, SIFT cloud, mean number of viewpoint features.
 */
class S2ObjectModel : public PointCloudObject
{
	public:
	/// Mean number of viewpoint features
	int mean_viewpoint_features_number;

	/// Cloud of SIFT - features extracted from RGB image and transformed from image into Cartesian space.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot;
};


//} //: namespace Types

#endif /* S2OBJECTMODEL_HPP_ */
