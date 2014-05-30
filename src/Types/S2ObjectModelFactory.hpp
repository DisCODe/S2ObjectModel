#ifndef S2OBJECTMODELFACTORY_HPP_
#define S2OBJECTMODELFACTORY_HPP_

#include <Types/AbstractObjectFactory.hpp> 
#include <Types/S2ObjectModel.hpp> 


/*!
 * \class S2ObjectModelFactory
 * \brief Factory responsible for production of SIFT and SHOT Object Models.
 */
class S2ObjectModelFactory : public AbstractObjectFactory
{

public:
	S2ObjectModelFactory(){
		mean_viewpoint_features_number = 0;
	}

	~S2ObjectModelFactory(){}

	/// Produces and returns a S2OM object.
	AbstractObject* produce(){
		S2ObjectModel *s2om = new S2ObjectModel;
		s2om->cloud_xyzrgb = cloud_xyzrgb;
		s2om->cloud_xyzsift = cloud_xyzsift;
		s2om->cloud_xyzshot = cloud_xyzshot;
		s2om->name = model_name;
		s2om->mean_viewpoint_features_number = mean_viewpoint_features_number;
		return s2om;
	}
	
protected:
	/// Cloud of XYZSIFT points - feature cloud.
	pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift;
	
	/// Cloud of XYZSHOT points - feature cloud.
	pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot;
	
	/// Cloud of XYZRGB points - object model cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb;

	/// Name of the model.
	std::string model_name;

	/// Mean number of viewpoint features.
	int mean_viewpoint_features_number;
	
};
#endif /* S2OBJECTMODELFACTORY_HPP_ */
