#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  
    double normals_radius= 0.01;
  if (pcl::console::parse (argc, argv, "-n", normals_radius) >= 0)
  {
    std::cout << " Radius: " << normals_radius << "\n";
  }
  
  PCL_INFO("1");

      double shot_radius = 0.05;
  if (pcl::console::parse (argc, argv, "-s", shot_radius) >= 0)
  {
    std::cout << " shot_radius: " << shot_radius << "\n";
  }
 
  PCL_INFO("2");
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
 //   PCL_ERROR ("Couldn't read file cloud.pcd \n");
 //   return (-1);
  }
  
  PCL_INFO(argv[1]);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *keypoints) == -1) //* load the file
  {
	  PCL_WARN ("Couldn't read file keypoints.pcd \n");
  }
  
  PCL_INFO("4");

  	NormalCloudPtr normals(new NormalCloud());
	pcl::NormalEstimationOMP<PointT, NormalT> est;
	est.setRadiusSearch(normals_radius);
	est.setInputCloud(cloud);
	est.compute(*normals);

	
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> green (cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (keypoints, 255, 0, 0);
	//////
  
	PCL_INFO("normals_radius: " + normals_radius);

   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, green, "cloud");
    viewer->addPointCloud<pcl::PointXYZRGB> (keypoints, red, "keypoints");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 3, 0.05, "normals");
  viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
   viewer->addSphere (cloud->points[10], normals_radius, "sphere+norm", 0);
 //   viewer->addSphere (cloud->points[0], shot_radius, "sphere+shot", 0);


  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
