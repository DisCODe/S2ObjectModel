#ifndef POINTXYZSHOT_HPP_
#define POINTXYZSHOT_HPP_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
//namespace Types {

struct PointXYZSHOT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float descriptor[352];
  float rf[9];
  int multiplicity; 
  int pointId;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
 
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZSHOT          // here we assume a XYZ + "test" (as fields)
                                   ,(float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[352], descriptor, descriptor)
                                   (float[9], rf, rf)
                                   (int, multiplicity, multiplicity)
                                   (int, pointId, pointId)
)


//} //: namespace Types

#endif /* POINTXYZSHOT_HPP_ */
