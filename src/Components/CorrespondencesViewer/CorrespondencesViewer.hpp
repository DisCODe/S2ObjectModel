/*!
 * \file
 * \brief 
 * \author Joanna,,,
 */

#ifndef CORRESPONDENCESVIEWER_HPP_
#define CORRESPONDENCESVIEWER_HPP_

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include "EventHandler2.hpp"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Types/PointXYZSHOT.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/HomogMatrix.hpp>


namespace Processors {
namespace CorrespondencesViewer {

/*!
 * \class CorrespondencesViewer
 * \brief CorrespondencesViewer processor class.
 *
 * CorrespondencesViewer processor.
 */
class CorrespondencesViewer: public Base::Component {
public:
	/*!
	 * Constructor.
	 */
	CorrespondencesViewer(const std::string & name = "CorrespondencesViewer");

	/*!
	 * Destructor
	 */
	virtual ~CorrespondencesViewer();

	/*!
	 * Prepare components interface (register streams and handlers).
	 * At this point, all properties are already initialized and loaded to 
	 * values set in config file.
	 */
	void prepareInterface();

protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	void display();
	void display2();
	void display3();
	void display4();
	void displayOnPort(int port, pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
			pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints,
			pcl::CorrespondencesPtr corrs,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud);
	void on_spin();

	pcl::visualization::PCLVisualizer * viewer;

	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondeces;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_source_keypoints;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_target_keypoints;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_source_cloud;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_target_cloud;

	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondeces2;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_source_keypoints2;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_target_keypoints2;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_source_cloud2;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_target_cloud2;

	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondeces3;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_source_keypoints3;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_target_keypoints3;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_source_cloud3;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_target_cloud3;

	Base::DataStreamIn<pcl::CorrespondencesPtr> in_correspondeces4;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_source_keypoints4;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr> in_target_keypoints4;

	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_source_cloud4;
	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> in_target_cloud4;
	
	Base::Property<bool> display_source_cloud;
	Base::Property<bool> display_target_cloud;
	Base::Property<bool> display_target_keypoints;
	Base::Property<bool> display_source_keypoints;
	Base::Property<bool> all_viewpoints;
	Base::Property<int> correspondences_size;

	int left;
	int right;

	int left_down;
	int right_down;
};

} //: namespace CorrespondencesViewer
} //: namespace Processors

/*
 * Register processor component.
 */
REGISTER_COMPONENT("CorrespondencesViewer", Processors::CorrespondencesViewer::CorrespondencesViewer)

#endif /* CORRESPONDENCESVIEWER_HPP_ */
