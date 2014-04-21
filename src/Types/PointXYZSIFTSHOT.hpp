#ifndef POINTXYZSIFTSHOT_HPP_
#define POINTXYZSIFTSHOT_HPP_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
//namespace Types {

struct PointXYZSIFTSHOT
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  float shot[352];
  float sift[128];
  float rf[9];
  int multiplicity; 
  int pointId;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
 
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZSIFTSHOT          // here we assume a XYZ + "test" (as fields)
                                   ,(float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float[352], shot, shot)
                                   (float[128], sift, sift)
                                   (float[9], rf, rf)
                                   (int, multiplicity, multiplicity)
                                   (int, pointId, pointId)
)


//} //: namespace Types

#endif /* POINTXYZSIFTSHOT_HPP_ */
