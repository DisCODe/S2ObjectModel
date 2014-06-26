#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d_omp.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalCloud;
typedef NormalCloud::Ptr NormalCloudPtr;

#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/susan.h>
#include <pcl/keypoints/iss_3d.h>
#include <iostream>

typedef pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI> HarrisKeypoint;
typedef pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI> HarrisKeypoint6D;


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{
  // --------------------------------------
  // -----Parse Command Line Arguments-----
  // --------------------------------------
  
    double normals_radius= 0.005;
  if (pcl::console::parse (argc, argv, "-n", normals_radius) >= 0)
  {
    std::cout << " Radius: " << normals_radius << "\n";
  }
  
      double harris_radius = 0.05;
  if (pcl::console::parse (argc, argv, "--hariss_radius", harris_radius) >= 0)
  {
    std::cout << " harris_radius: " << harris_radius << "\n";
  }
 
      double harris_search_radius = 0.05;
  if (pcl::console::parse (argc, argv, "--harris_search_radius", harris_search_radius) >= 0)
  {
    std::cout << " harris_search_radius: " << harris_search_radius << "\n";
  }

  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloudRGB) == -1) //* load the file
  {
    std::cout << "Couldn't read file cloud.pcd ";
    return (-1);
  }
  
  pcl::copyPointCloud(*cloudRGB, *cloud);

      pcl::PointCloud<pcl::PointXYZ>::Ptr harris3d (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr iss3d (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr susan (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr harris6d (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr sift (new pcl::PointCloud<pcl::PointXYZ>);

  
  	NormalCloudPtr normals(new NormalCloud());
	pcl::NormalEstimationOMP<PointT, NormalT> est;
	est.setRadiusSearch(normals_radius);
	est.setInputCloud(cloud);
	est.compute(*normals);

	{

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(
				new pcl::PointCloud<pcl::PointXYZI>());

		HarrisKeypoint* detector = new HarrisKeypoint(HarrisKeypoint::HARRIS);

		detector->setNonMaxSupression(true);
		detector->setRadius(harris_radius);
		detector->setRadiusSearch(harris_search_radius);
		detector->setMethod(HarrisKeypoint::HARRIS);
	//	detector->setKSearch();
		detector->setNumberOfThreads(10);
		detector->setSearchMethod(tree);
	//	detector->setThreshold();
	//	detector->use_indices_ = false;
		detector->setInputCloud(cloud);


		detector->compute(*keypoints_temp);

		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(
				new pcl::PointCloud<pcl::PointXYZ>());

		pcl::copyPointCloud(*keypoints_temp, *harris3d);
}

	std::cout <<  " harris3d size :" << harris3d->size();


	{

		double iss_salient_radius_;
			double iss_non_max_radius_;
			double iss_gamma_21_ (0.975);
			double iss_gamma_32_ (0.975);
			double iss_min_neighbors_ (20);
			int iss_threads_ (10);

			pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

			// Fill in the model cloud

			double model_resolution = 0.01;

			// Compute model_resolution

			iss_salient_radius_ = 6 * model_resolution;
			iss_non_max_radius_ = 4 * model_resolution;

			//
			// Compute keypoints
			//
			pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;

			iss_detector.setSearchMethod (tree);
			iss_detector.setSalientRadius (iss_salient_radius_);
			iss_detector.setNonMaxRadius (iss_non_max_radius_);
			iss_detector.setThreshold21 (iss_gamma_21_);
			iss_detector.setThreshold32 (iss_gamma_32_);
			iss_detector.setMinNeighbors (iss_min_neighbors_);
			iss_detector.setNumberOfThreads (iss_threads_);
			iss_detector.setInputCloud (cloud);
			iss_detector.compute (*iss3d);
	}


	std::cout << " iss3d size :" << iss3d->size() << std::endl;

	{
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

		pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>* susan3D = new  pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB>;
		susan3D->setInputCloud(cloudRGB);
		susan3D->setNonMaxSupression(false);
		susan3D->setSearchMethod(tree);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZRGB> ());
		susan3D->compute(*keypoints);

		pcl::PointCloud<pcl::PointXYZ>::Ptr susan(new pcl::PointCloud<pcl::PointXYZ>());

		pcl::copyPointCloud(*keypoints, *susan);
	}

	std::cout <<  " susan size :" << susan->size() << std::endl;

	{

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());

		pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp(
				new pcl::PointCloud<pcl::PointXYZI>());

		HarrisKeypoint6D* detector = new HarrisKeypoint6D(HarrisKeypoint::HARRIS);

		detector->setNonMaxSupression(true);
		detector->setRadius(harris_radius);
		detector->setRadiusSearch(harris_search_radius);
	//	detector->setMethod(HarrisKeypoint::HARRIS);
	//	detector->setKSearch();
		detector->setNumberOfThreads(10);
		detector->setSearchMethod(tree);
	//	detector->setThreshold();
	//	detector->use_indices_ = false;
		detector->setInputCloud(cloudRGB);


		detector->compute(*keypoints_temp);

		pcl::copyPointCloud(*keypoints_temp, *harris6d);
}

	std::cout <<  " harris6d size :" << harris6d->size() << std::endl;

	

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white (cloud, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (harris3d, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green (iss3d, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (susan, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col2 (harris6d, 255, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> col3 (sift, 255, 255, 0);
	//////
  
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, white, "cloud");
    viewer->addPointCloud<pcl::PointXYZ> (harris3d, red, "keypoints2");
    viewer->addPointCloud<pcl::PointXYZ> (iss3d, green, "keypoints3");
    viewer->addPointCloud<pcl::PointXYZ> (susan, blue, "keypoint4s");
    viewer->addPointCloud<pcl::PointXYZ> (harris6d, col2, "keypoin3ts");
    viewer->addPointCloud<pcl::PointXYZ> (sift, col3, "keypoint36");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "keypoints2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "keypoints3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "keypoint4s");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "keypoin3ts");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "keypoint36");
 // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 5, 0.05, "normals");
  viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
//   viewer->addSphere (cloud->points[0], normals_radius, "sphere+norm", 0);
  //  viewer->addSphere (cloud->points[0], shot_radius, "sphere+shot", 0);


  //--------------------
  // -----Main loop-----
  //--------------------
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
