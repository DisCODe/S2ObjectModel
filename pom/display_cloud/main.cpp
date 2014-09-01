#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// --------------
// -----Main-----
// --------------
int main(int argc, char** argv) {
	// --------------------------------------
	// -----Parse Command Line Arguments-----
	// --------------------------------------

	double normals_radius = 0.03;
	double harris_radius = 0.5;
	double harris_search_radius = 0.01;
	double model_resolution = 0.001;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGBA>);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(argv[1], *cloudRGB) == -1) //* load the file
			{
		std::cout << "Couldn't read file cloud.pcd ";
		return (-1);
	}

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloudRGB);
	viewer->addPointCloud<pcl::PointXYZRGBA>(cloudRGB, rgb, "sample cloud");

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}
